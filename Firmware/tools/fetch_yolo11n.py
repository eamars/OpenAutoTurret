"""Fetch the reviewed official YOLO11n comparison artifact into project-local run/models.

Use a project Python venv. The AGPL-3.0 model remains an optional comparison profile.
"""
import hashlib
from pathlib import Path
import urllib.request

COMMIT = 'ddfe4c7ec96c0289e5f2d5996894311a218b2e1c'
SHA256 = 'c8e53dd9208debff3cd72044600095624952d6fb4e67910e2e8098251e0307fa'
NAME = 'imx500_network_yolo11n_pp.rpk'
URL = f'https://raw.githubusercontent.com/raspberrypi/imx500-models/{COMMIT}/{NAME}'


def main():
    destination = Path(__file__).resolve().parents[2]/'run/models'/NAME
    if destination.exists() and hashlib.sha256(destination.read_bytes()).hexdigest() == SHA256:
        print(f'Already verified: {destination}')
        return
    with urllib.request.urlopen(URL, timeout=30) as response:
        data = response.read(16*1024*1024)
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise ValueError('Downloaded bytes do not match the reviewed artifact')
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix('.rpk.part')
    temporary.write_bytes(data)
    temporary.replace(destination)
    print(f'Verified {destination}; SHA256 {SHA256}; upstream license AGPL-3.0')


if __name__ == '__main__':
    main()
