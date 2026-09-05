"""
The Dockerfile must build, and it must not be a second copy of requirements.txt.
================================================================================
Nothing else in this suite touches the container: every test here is pure text
parsing, so it runs in the blocking `unit` CI job on a host with no Docker.
That matters because the alternative -- finding out at `Reopen in Container`
time -- costs a ten-minute image build per attempt.

Both checks below exist because the mistake they catch was actually made.

    the comment inside a RUN ....... a line of prose was added mid-`RUN`, after
                                     an `&&`. Docker's parser deletes comment
                                     LINES but leaves the `&&` that preceded
                                     them, so the shell received `... &&` and
                                     died with "unexpected end of file". The
                                     Dockerfile is not executed by anything on
                                     this host, so nothing caught it until a
                                     human tried to open the container.
    the second dependency list .... the image pip-installed its own hand-kept
                                     list. It drifted from requirements.txt and
                                     lost pillow (tvc.py record) and pytest --
                                     so the container could not run this suite.

The Dockerfile parsing here mirrors Docker's own two rules, in this order:
comment lines are removed first, THEN backslash continuations are joined. Doing
it the other way round hides exactly the bug the first test is looking for.
"""
import os
import re
import shutil
import subprocess

import pytest

# Packages a `<depend>` may name without the Dockerfile installing anything:
# ros:jazzy is the ros_core/ros_base image, so the client libraries, the common
# message packages and the ament/rosidl build tooling are already present.
# A dep NOT in this set has to be installed explicitly or `colcon build` fails
# on a machine that is otherwise correct -- which reads as a code error.
_BASE_IMAGE_PROVIDES = {
    "rclpy", "rclcpp",
    "std_msgs", "sensor_msgs", "geometry_msgs", "nav_msgs", "rosgraph_msgs",
    "builtin_interfaces",
    "ament_cmake", "ament_lint_auto", "ament_lint_common",
    "ament_copyright", "ament_flake8", "ament_pep257",
    "rosidl_default_generators", "rosidl_default_runtime",
    "python3-pytest",
}

# Packages installed by pip from requirements.txt, so a `<depend>` naming the
# rosdep spelling of one of them is satisfied even though no apt package is.
_PIP_SATISFIES = {"python3-numpy": "numpy", "python3-scipy": "scipy"}


def _dockerfile(repo):
    return os.path.join(repo, "Dockerfile")


def _instructions(repo):
    """Return the Dockerfile as (line_number, joined_text) logical instructions.

    Applies Docker's parsing rules in Docker's order: strip whole-line comments,
    then fold `\\` continuations. The line number is that of the instruction's
    first physical line, so a failure points at something a human can find.
    """
    with open(_dockerfile(repo), encoding="utf-8") as f:
        raw = f.read().split("\n")

    kept = [(n, ln) for n, ln in enumerate(raw, 1) if not ln.lstrip().startswith("#")]

    out, pending, start = [], None, None
    for n, ln in kept:
        body = ln[:-1] if ln.rstrip().endswith("\\") else ln
        cont = ln.rstrip().endswith("\\")
        if pending is None:
            if not ln.strip():
                continue
            pending, start = body, n
        else:
            pending += " " + body.strip()
        if not cont:
            out.append((start, " ".join(pending.split())))
            pending = None
    if pending is not None:
        out.append((start, " ".join(pending.split())))
    return out


def _runs(repo):
    return [(n, t[len("RUN "):]) for n, t in _instructions(repo) if t.startswith("RUN ")]


# --- the Dockerfile is syntactically a Dockerfile -----------------------------

def test_no_run_has_an_operator_with_nothing_after_it(repo):
    """`&& \\` followed only by comment lines leaves a dangling `&&`.

    This is the exact failure mode of a comment written inside a RUN. Checked
    structurally rather than by running a shell, so it works everywhere."""
    for n, cmd in _runs(repo):
        assert not re.search(r"(&&|\|\||;)\s*$", cmd), (
            "Dockerfile:%d ends a RUN with a dangling operator -- a comment "
            "line inside the continuation was deleted by Docker's parser and "
            "took its command with it. Move the comment above the RUN.\n  %s"
            % (n, cmd[-120:]))
        assert not re.search(r"(&&|\|\||;)\s*(&&|\|\|)", cmd), (
            "Dockerfile:%d has two operators in a row, so a command between "
            "them was removed -- almost certainly a comment line inside the "
            "RUN.\n  %s" % (n, cmd))


_KEYWORDS = {
    "FROM", "RUN", "CMD", "LABEL", "MAINTAINER", "EXPOSE", "ENV", "ADD", "COPY",
    "ENTRYPOINT", "VOLUME", "USER", "WORKDIR", "ARG", "ONBUILD", "STOPSIGNAL",
    "HEALTHCHECK", "SHELL",
}


def test_every_instruction_starts_with_a_dockerfile_keyword(repo):
    """The strongest pure-Python detector of a comment written inside a RUN.

    The comment's FIRST line usually still carries the `&&` (it reads
    `&& # prose`), so that line stays and terminates the instruction; the
    `#`-prefixed lines after it are deleted; and the continuation that follows
    them is left stranded as an instruction of its own, beginning with `&&`.
    Docker reports this as an unknown instruction, several lines away from the
    line a human would look at."""
    for n, txt in _instructions(repo):
        kw = txt.split()[0]
        assert kw in _KEYWORDS, (
            "Dockerfile:%d is not a Dockerfile instruction: %r\n"
            "This is what a comment written INSIDE a RUN looks like after the "
            "parser removes the comment lines -- the rest of the RUN is left "
            "stranded. Move the comment above the RUN." % (n, txt[:100]))


def test_no_instruction_ends_the_file_mid_continuation(repo):
    """A trailing `\\` on the last line silently swallows the instruction."""
    with open(_dockerfile(repo), encoding="utf-8") as f:
        text = f.read()
    assert not text.rstrip().endswith("\\"), "Dockerfile ends with a continuation"


@pytest.mark.skipif(shutil.which("bash") is None, reason="no bash on this host")
def test_every_run_is_valid_shell(repo):
    """`bash -n` parses each RUN without executing it.

    Broader than the structural checks above -- it also catches an unbalanced
    quote or an unterminated `if` -- but it needs a shell, so the structural
    tests stay as the ones that always run."""
    for n, cmd in _runs(repo):
        r = subprocess.run(["bash", "-n", "-c", cmd], capture_output=True, text=True)
        assert r.returncode == 0, "Dockerfile:%d is not valid shell:\n%s" % (n, r.stderr)


# --- one dependency list, not two --------------------------------------------

def _requirements(repo):
    """The distribution names in requirements.txt, comments stripped."""
    names = []
    with open(os.path.join(repo, "requirements.txt"), encoding="utf-8") as f:
        for ln in f:
            ln = ln.split("#")[0].strip()
            if ln:
                names.append(re.split(r"[<>=!~\[]", ln)[0].strip().lower())
    return names


def test_the_image_installs_requirements_txt(repo):
    """The container must get its Python stack from the same file the host does,
    or the two environments differ in a way no test would notice."""
    text = open(_dockerfile(repo), encoding="utf-8").read()
    assert "requirements.txt" in text, (
        "the Dockerfile does not reference requirements.txt -- if it keeps its "
        "own list, the two drift and the container silently lacks a package")
    assert any("-r /tmp/requirements.txt" in cmd for _, cmd in _runs(repo)), (
        "requirements.txt is mentioned but never installed with `pip install -r`")


def test_the_image_does_not_keep_a_second_copy_of_the_list(repo):
    """A package named in requirements.txt must not also be a literal argument
    to a pip install in the Dockerfile. That is the duplicate that drifted."""
    reqs = set(_requirements(repo))
    for n, cmd in _runs(repo):
        for pip in re.findall(r"pip3? install\s+(.*?)(?:&&|$)", cmd):
            if "-r " in pip:
                continue
            args = {a.lower() for a in pip.split() if not a.startswith("-")}
            dup = sorted(args & reqs)
            assert not dup, (
                "Dockerfile:%d pip-installs %s, which requirements.txt already "
                "lists. Remove it here; requirements.txt is the source."
                % (n, ", ".join(dup)))


# --- declared dependencies are actually installed -----------------------------

def _declared_deps(repo):
    """Every <depend> in src/*, minus the packages this workspace itself builds.

    tvc_control depends on tvc_msgs; colcon supplies that, not apt."""
    deps, ours = set(), set()
    src = os.path.join(repo, "src")
    for pkg in sorted(os.listdir(src)):
        path = os.path.join(src, pkg, "package.xml")
        if os.path.exists(path):
            text = open(path, encoding="utf-8").read()
            deps |= set(re.findall(r"<(?:\w*_)?depend>([^<]+)</", text))
            ours |= set(re.findall(r"<name>([^<]+)</name>", text))
    return {d.strip() for d in deps} - {o.strip() for o in ours}


def test_every_declared_dependency_can_be_resolved(repo):
    """A `<depend>` nothing installs fails `colcon build` on a correct machine.

    `actuator_msgs` sat in package.xml unresolved for the whole time the ROS 2
    path had never been built; it arrived only as a transitive dependency of
    ros_gz_bridge, which is luck, not a dependency."""
    text = open(_dockerfile(repo), encoding="utf-8").read()
    reqs = set(_requirements(repo))
    missing = []
    for dep in sorted(_declared_deps(repo)):
        if dep in _BASE_IMAGE_PROVIDES:
            continue
        if _PIP_SATISFIES.get(dep) in reqs:
            continue
        if dep in text or "ros-jazzy-" + dep.replace("_", "-") in text:
            continue
        missing.append(dep)
    assert not missing, (
        "declared in a package.xml but installed by nothing in the Dockerfile: "
        "%s. Either add the apt package, or -- if ros:jazzy already ships it -- "
        "add it to _BASE_IMAGE_PROVIDES in this file with that reason."
        % ", ".join(missing))
