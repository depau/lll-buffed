import subprocess
from pathlib import Path

from hatchling.metadata.plugin.interface import MetadataHookInterface


class CustomBuildHook(MetadataHookInterface):
    def update(self, metadata: dict) -> None:
        requirements = Path.cwd() / "ratos-klipper/scripts/klippy-requirements.txt"
        if requirements.exists():
            return

        print("Pulling git submodules...")
        subprocess.run(
            ["git", "submodule", "update", "--init", "--recursive"],
            check=True,
        )
