"""Compare 50k whole plans, frozen grants and state with the exact round-17 source."""
from pathlib import Path
import subprocess,tempfile,re,os,shlex,hashlib
here=Path(__file__).resolve().parent;root=here.parents[2]
source=(here/'cedeea0_coord.inc').read_bytes()
with tempfile.TemporaryDirectory() as d:
 p=Path(d);(p/'old.c').write_bytes(source)
 names=sorted(set(re.findall(rb'\brelay_coord_\w+\b',source)))
 (p/'rename.h').write_text('\n'.join('#define '+n.decode()+' cedeea0_'+n.decode() for n in names)+'\n')
 cc=shlex.split(os.environ.get('CC','cc'));flags=['-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined','-I'+str(root/'src/connection')]
 subprocess.run(cc+flags+['-include',str(p/'rename.h'),'-c',str(p/'old.c'),'-o',str(p/'old.o')],check=True)
 subprocess.run(cc+flags+['-DCOMPARE_CEDEEA0',str(here/'test_split.c'),str(root/'src/connection/relay_coord.c'),str(p/'old.o'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
print('cedeea0 exact source SHA256:',hashlib.sha256(source).hexdigest())
print('Production: 50000 exact cedeea0 preplans, grants and coordinator state comparisons passed')
