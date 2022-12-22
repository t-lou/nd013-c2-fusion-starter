import os
import subprocess

def count_files(directory: str, pattern: str) -> int:
    """ Count the number of files in one directory.
    Params:
        directory: the target directory
        pattern: the bash pattern for matching
    """
    p = subprocess.Popen(f'find -name "{pattern}" | wc -l',
                         stdout=subprocess.PIPE,
                         shell=True,
                         cwd=directory)
    out, _ = p.communicate()
    return int(out.strip())
