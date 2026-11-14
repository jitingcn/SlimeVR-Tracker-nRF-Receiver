"""Compare compact snapshot policy with the pre-change full coordinator."""
from pathlib import Path
import os, re, shlex, subprocess, tempfile
here = Path(__file__).resolve().parent
root = here.parents[2]
source = subprocess.check_output(['git', 'show', 'b4dac52:src/connection/relay_coord.c'], cwd=root)
with tempfile.TemporaryDirectory() as directory:
    p = Path(directory)
    (p / 'old.c').write_bytes(source)
    names = sorted(set(re.findall(rb'\brelay_coord_\w+\b', source)))
    (p / 'rename.h').write_text('#define RC_ID_OUTSIDE(id) 0\n#define RC_MEMBER_MASK 65535U\n'+'\n'.join('#define '+n.decode()+' baseline_'+n.decode() for n in names)+'\n')
    cc = shlex.split(os.environ.get('CC', 'cc'))
    flags = ['-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
             '-I'+str(root / 'src/connection')]
    subprocess.run(cc+flags+['-include', str(p/'rename.h'), '-c', str(p/'old.c'), '-o', str(p/'old.o')], check=True)
    subprocess.run(cc+flags+[str(here/'test_snapshot_equivalence.c'), str(root/'src/connection/relay_coord.c'),
                             str(p/'old.o'), '-o', str(p/'test')], check=True)
    subprocess.run([str(p/'test')], check=True)
