#!/usr/bin/env python3
"""Select optimized/legacy Sharpa behavior for the next driver start."""
import argparse
import json
import os
from pathlib import Path
import tempfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('status','legacy','optimized'))
    args=parser.parse_args()
    path=Path(__file__).resolve().parents[2]/'config/sharpa_runtime.json'
    if args.mode!='status':
        with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,delete=False) as stream:
            temporary=Path(stream.name)
            json.dump({'enabled':args.mode=='optimized'},stream,indent=2)
            stream.write('\n');stream.flush();os.fsync(stream.fileno())
        os.chmod(temporary,0o644)
        os.replace(temporary,path)
    enabled=json.loads(path.read_text()).get('enabled') is True
    print('Sharpa next-start mode: '+('optimized' if enabled else 'legacy'))
    print('Restart the existing teleoperation entry to apply. Running device processes are unchanged.')
    if 'SHARPA_RUNTIME_OPTIMIZATION' in os.environ:
        print('Environment override SHARPA_RUNTIME_OPTIMIZATION='+os.environ['SHARPA_RUNTIME_OPTIMIZATION'])


if __name__=='__main__':main()
