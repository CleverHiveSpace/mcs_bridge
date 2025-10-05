from setuptools import setup, find_packages

setup(
    name="mcs-bridge",
    version="0.1.0",
    description="MCS Bridge - Send telemetry data to MCS backend",
    author="Your Name",
    author_email="your.email@example.com",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "bidict==0.23.1",
        "certifi==2025.10.5",
        "charset-normalizer==3.4.3",
        "h11==0.16.0",
        "idna==3.10",
        "python-dotenv==1.1.1",
        "python-engineio==4.12.3",
        "python-socketio==5.14.1",
        "requests==2.32.5",
        "simple-websocket==1.1.0",
        "urllib3==2.5.0",
        "websocket-client==1.8.0",
        "wsproto==1.2.0",
    ],
    entry_points={
        "console_scripts": [
            "mcs-bridge=mcs_bridge.__main__:main",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
    ],
)
