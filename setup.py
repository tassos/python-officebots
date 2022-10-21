import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="officebots",
    version="0.3.0",
    author="Séverin Lemaignan",
    author_email="severin.lemaignan@brl.ac.uk",
    description="Python API for the OfficeBots game",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://git.brl.ac.uk/s-lemaignan/officebots-python",
    package_dir={"": "src"},
    packages=["officebots"],
    scripts=["scripts/officebots-ros"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
)
