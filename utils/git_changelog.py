import subprocess


def get_commit_hash(commit_desc: str) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "--short", commit_desc], capture_output=True, text=True
    )

    if result.returncode != 0:
        raise Exception(
            f"git command {result.args} failed with error code {result.returncode}: {result.stderr}"
        )

    return result.stdout.replace("\n", "")


def get_last_tag() -> str:
    result = subprocess.run(
        ["git", "describe", "--tags", "--abbrev=0"], capture_output=True, text=True
    )

    if result.returncode != 0:
        raise Exception(
            f"git command {result.args} failed with error code {result.returncode}: {result.stderr}"
        )

    return result.stdout.replace("\n", "")


def get_commit_diff(hash_from: str, hash_to: str) -> list[str]:

    result = subprocess.run(
        [
            "git",
            "log",
            "--oneline",
            "--decorate",
            "--decorate-refs=no",
            f"{hash_from}...{hash_to}",
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise Exception(
            f"git command {result.args} failed with error code {result.returncode}: {result.stderr}"
        )

    return result.stdout.split("\n")


last_tag = get_last_tag()
commit_last_tag = get_commit_hash(last_tag)
commit_current = get_commit_hash("HEAD")
commit_diff_log = get_commit_diff(commit_last_tag, commit_current)

print(f"Changelog from {last_tag} ({commit_last_tag}) to HEAD ({commit_current})")
print("\n".join(commit_diff_log))
