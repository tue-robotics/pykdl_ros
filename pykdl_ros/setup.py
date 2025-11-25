from pathlib import Path

from setuptools import find_packages, setup

package_name = "pykdl_ros"

package_share = Path("share", package_name)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            Path("share", "ament_index", "resource_index", "packages").as_posix(),
            [Path("resource", package_name).as_posix()],
        ),
        (package_share.as_posix(), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Matthijs van der Burgh",
    maintainer_email="MatthijsBurgh@outlook.com",
    description="Stamped PyKDL classes",
    license="BSD",
    extras_require={"test": ["pytest"]},
)
