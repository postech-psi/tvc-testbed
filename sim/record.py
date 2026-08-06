"""
Capture frames from the world's chase camera and write an animated GIF.

Runs alongside hover.py against the same simulator. Headless rendering is
used because the dev container has no display; there is no ffmpeg in the
image either, so frames are assembled with PIL into a GIF rather than a
video container.

    python3 sim/record.py --duration 20 --out sim/hover.gif
"""
import argparse
import sys
import time

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

frames = []
_last = [0.0]


def make_cb(every):
    def cb(msg):
        now = time.time()
        if now - _last[0] < every:
            return
        _last[0] = now
        frames.append((msg.width, msg.height, bytes(msg.data)))
    return cb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--out", default="sim/hover.gif")
    ap.add_argument("--fps", type=float, default=12.0, help="frames captured per second")
    ap.add_argument("--scale", type=float, default=0.6)
    args = ap.parse_args()

    node = Node()
    if not node.subscribe(Image, "/chase_cam/image", make_cb(1.0 / args.fps)):
        print("could not subscribe to /chase_cam/image")
        return 1

    print("recording %.0f s from /chase_cam/image ..." % args.duration, flush=True)
    end = time.time() + args.duration
    while time.time() < end:
        time.sleep(0.05)

    if not frames:
        print("NO FRAMES. The camera sensor needs the Sensors system plus a\n"
              "working render engine; headless rendering can fail silently in a\n"
              "container with no GPU. Try `gz sim -s -r --headless-rendering`.")
        return 2

    from PIL import Image as PImage
    imgs = []
    for w, h, data in frames:
        if len(data) < w * h * 3:
            continue
        im = PImage.frombytes("RGB", (w, h), data[:w * h * 3])
        if args.scale != 1.0:
            im = im.resize((int(w * args.scale), int(h * args.scale)),
                           PImage.LANCZOS)
        imgs.append(im)

    if not imgs:
        print("frames arrived but none decoded")
        return 2

    imgs[0].save(args.out, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / args.fps), loop=0, optimize=True)
    print("wrote %s (%d frames, %dx%d)" % (args.out, len(imgs),
                                           imgs[0].width, imgs[0].height))
    return 0


if __name__ == "__main__":
    sys.exit(main())
