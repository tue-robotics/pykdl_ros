from os import path

from setuptools import find_packages, setup

package_name = "tf2_pykdl_ros"

package_share = path.join("share", package_name)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            path.join("share", "ament_index", "resource_index", "packages"),
            [path.join("resource", package_name)],
        ),
        (package_share, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Matthijs van der Burgh",
    maintainer_email="matthijs.van.der.burgh@vbti.nl",
    description="tf2 conversions for pykdl_ros",
    license="BSD",
    extras_require={"test": ["pytest"]},
)
