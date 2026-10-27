from pathlib import Path
import hashlib,json,urllib.request
W=Path(__file__).resolve().parents[1]
for row in json.loads((W/'config/resources_manifest.json').read_text()):
 p=W/'src/FALCON'/row['relative_to_FALCON']
 if p.exists() and hashlib.sha256(p.read_bytes()).hexdigest()==row['sha256']:continue
 data=urllib.request.urlopen(row['url'],timeout=90).read()
 assert hashlib.sha256(data).hexdigest()==row['sha256'], row['url']
 p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
 print(p)
