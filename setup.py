from setuptools import setup, find_packages

setup(
    name="mcs-bridge",
    packages=find_packages(),
    entry_points={
        "console_scripts": [
            "mcs-bridge=mcs_bridge.__main__:main",
            "mcs-video-uploader=mcs_video_uploader.__main__:main",
        ],
    },
)
