"""Download the book's photographs from Wikimedia Commons and write their credits.

Usage: python3 book/tools/photos.py
Writes book/photos/<key>.jpg and book/photos/credits.tex (\\photocredit{key}{...} macros).
Every file is public domain or under a Creative Commons licence; the credits page lists
author, licence and source for each.
"""

import html
import io
import json
import re
import time
import urllib.parse
import urllib.request
from pathlib import Path

from PIL import Image

UA = {"User-Agent": "AnemostatosBook/1.0 (stelmaszewskiw@gmail.com)"}
OUT = Path(__file__).resolve().parent.parent / "photos"

# key -> Commons file title (exact) or a prefix of it (resolved by search)
PHOTOS = {
    "governor": "File:Boulton and Watt centrifugal governor-MJ.jpg",
    "tacoma": "File:Tacoma Narrows still.png",
    "turbinehall": "File:VIEW OF TURBINE HALL LOOKING SOUTHWEST AT WESTINGHOUSE-PARSONS TURBINE NUMB",
    "taipei": "File:Taipei 101 Tuned Mass Damper 2010.jpg",
    "newmexico": "File:USS New Mexico (BB-40) at anchor in 1920.jpg",
    "cruise": "File:Cruise control Mercedes C220.jpg",
    "thermostat": "File:Honeywell round thermostat.jpg",
    "mpu6050": "File:IvenSense 3-Axis-Gyro-Accelerometer-IC MPU-6050 10937-01.jpg",
    "f8": "File:F-8 Digital Fly-by-Wire (DFBW) in flight over snow capped mountains DVIDS69",
    "dsky": "File:Apollo display and keyboard unit (DSKY) used on F-8 DFBW DVIDS683588.jpg",
    "curiosity": "File:Curiosity rover selfie at Namib Dune Sol 1128 (53678107023).jpg",
    "x29": "File:X-29 in Flight - GPN-2002-000193.jpg",
    "nwtc": "File:National Wind Technology Center - Colorado.jpg",
    "ingenuity": "File:Ingenuity is tested at the Kennedy Space Center.jpg",
    "hummingbird": "File:Hovering Hummingbird (14900025010).jpg",
    "distillation": "File:Colonne distillazione.jpg",
    "cheetah": "File:Cheetah Run.jpg",
    "kalman": "File:ETH-BIB-Kalman, Rudolf E. (1930-2016)-HK 04-01925.jpg",
    "segway": "File:Segway PT (2006).jpg",
    "apollo15": "File:View of the Apollo 15 Command-Service Module in lunar orbit (as15-88-11974)",
    "gps": "File:GPS-IIF-11.jpg",
    "phlab": "File:Cessna Citation II ‘PH-LAB’ (49295064688).jpg",
    "apolloimu": "File:Inertial Measurement Unit, Apollo Guidance and Navigation System, fabricate",
    "droneshow": "File:US flag at drone light show.jpg",
    "coaster": "File:Loop Coaster MOMOnGA vertical loop - Yomiuriland.jpg",
    "falcon9": "File:Falcon 9 first stage landing on Droneship.jpg",
    "racingdrone": "File:Racing Drone.jpg",
    "rally": "File:Kris Meeke Rally Finland 2016 Äänekoski–Valtra.JPG",
    "a320": "File:Airbus A320-214 Vueling EC-HHA cockpit (5508849819).jpg",
    "windtunnel": "File:MEDIUM CLOSE UP OF FAN IN FULL-SCALE WIND TUNNEL.tiff",
    "swift": "File:Drone racing – AI piloted drone vs human champions.webp",
    "knife": "File:Swiss army knife open 20050612 (cropped).jpg",
    "hubble": "File:Hubble Released 2009.jpg",
    "minorsky": "File:Portrait of Nicolas Minorsky.jpg",
    "ctesibius": "File:ARAGO Francois Astronomie Populaire T1 page 0067 Fig16-17.jpg",
    "maxwell": "File:James Clerk Maxwell, G.J. Stodart, 1890.jpg",
    "wiener": "File:Norbert Wiener.png",
    "bode": "File:Hendrik Wade Bode.png",
    "pontryagin": "File:Понтрягин Лев Семёнович.jpg",
}

# Fractions to trim (left, top, right, bottom): scan borders and printed captions.
CROPS = {
    "f8": (0, 0, 0, 0.10),
    "turbinehall": (0.035, 0.04, 0.05, 0.04),
    "windtunnel": (0.03, 0.03, 0.03, 0.03),
}


def api(**kw):
    kw["format"] = "json"
    url = "https://commons.wikimedia.org/w/api.php?" + urllib.parse.urlencode(kw)
    for attempt in range(5):
        try:
            return json.load(urllib.request.urlopen(urllib.request.Request(url, headers=UA)))
        except Exception:
            time.sleep(2 + 3 * attempt)
    raise RuntimeError(url)


def resolve(title):
    d = api(action="query", titles=title, prop="imageinfo")
    page = next(iter(d["query"]["pages"].values()))
    if "missing" not in page and "invalid" not in page:
        return title
    d = api(action="query", list="allimages", aiprefix=title[5:].replace(" ", "_"), ailimit=5)
    for hit in d["query"]["allimages"]:
        return "File:" + hit["name"].replace("_", " ")
    raise KeyError(title)


def clean(s):
    s = re.sub(r"<[^>]+>", "", s or "")
    s = html.unescape(s).strip()
    return re.sub(r"\s+", " ", s)


def tex(s):
    for a, b in [("\\", r"\textbackslash{}"), ("&", r"\&"), ("%", r"\%"), ("$", r"\$"),
                 ("#", r"\#"), ("_", r"\_"), ("{", r"\{"), ("}", r"\}"), ("~", r"\~{}"),
                 ("^", r"\^{}")]:
        s = s.replace(a, b)
    return s


def main():
    OUT.mkdir(exist_ok=True)
    credits = {}
    cache = OUT / "credits.json"
    if cache.exists():
        credits = json.loads(cache.read_text())
    for key, title in PHOTOS.items():
        target = OUT / f"{key}.jpg"
        if target.exists() and key in credits:
            continue
        title = resolve(title)
        d = api(action="query", titles=title, prop="imageinfo",
                iiprop="url|extmetadata|size", iiurlwidth=2200)
        ii = next(iter(d["query"]["pages"].values()))["imageinfo"][0]
        m = ii["extmetadata"]
        src = ii.get("thumburl") or ii["url"]
        raw = urllib.request.urlopen(urllib.request.Request(src, headers=UA)).read()
        img = Image.open(io.BytesIO(raw)).convert("RGB")
        if img.width > 2200:
            img = img.resize((2200, round(img.height * 2200 / img.width)), Image.LANCZOS)
        if key in CROPS:
            l, t, r, b = CROPS[key]
            w, h = img.size
            img = img.crop((int(w * l), int(h * t), int(w * (1 - r)), int(h * (1 - b))))
        img.save(target, quality=86, optimize=True, progressive=True)
        credits[key] = {
            "title": title,
            "artist": clean(m.get("Artist", {}).get("value")) or "Unknown",
            "credit": clean(m.get("Credit", {}).get("value")),
            "license": clean(m.get("LicenseShortName", {}).get("value")),
            "page": ii["descriptionurl"],
            "size": [img.width, img.height],
        }
        print(f"{key:12s} {img.width}x{img.height} {credits[key]['license']}  {title}")
        cache.write_text(json.dumps(credits, indent=1, ensure_ascii=False))
        time.sleep(0.5)
    lines = []
    for key in PHOTOS:
        c = credits[key]
        artist = c["artist"][:200]
        lines.append(
            "\\photocredit{%s}{%s}{%s}{\\url{%s}}" % (key, tex(artist), tex(c["license"]), c["page"].replace("%", r"\%"))
        )
    (OUT / "credits.tex").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
