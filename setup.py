from setuptools import setup, find_packages

setup(
    name="mcs-bridge",
    packages=find_packages(),
    entry_points={
        "console_scripts": [
            "mcs-bridge=mcs_bridge.__main__:main",
        ],
    },
)
