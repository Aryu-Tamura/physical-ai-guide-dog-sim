"""Check the pinned upstream assets before loading learned policies."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    checked=0
    for manifest in ['download_manifest.json','local_model_manifest.json']:
        for item in json.loads((ROOT/manifest).read_text())['files']:
            path=ROOT/item['path']
            assert path.is_file(), f'Missing: {path}'
            actual=hashlib.sha256(path.read_bytes()).hexdigest()
            assert actual==item['sha256'], f'Hash mismatch: {path}'
            checked+=1
    print(f'PASS: {checked} pinned asset files verified')

if __name__=='__main__':main()
