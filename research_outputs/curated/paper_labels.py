"""Author-year display labels; corpus IDs remain internal record keys."""

import json
from pathlib import Path


def first_author_labels():
    corpus_path = Path(__file__).resolve().parent / "curated_corpus.json"
    papers = json.loads(corpus_path.read_text(encoding="utf-8"))["main"]
    return {
        paper["ID"]: f"{paper['Citation'].split()[0].rstrip(',')} ({paper['Year']})"
        for paper in papers
    }
