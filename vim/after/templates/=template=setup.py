from setuptools import find_packages
from setuptools import setup


def get_install_requires():
    install_requires = []
    with open("requirements.txt") as f:
        for req in f:
            if req.startswith("-"):
                continue
            req.split("#")[0]
            install_requires.append(req)
    return install_requires


def main():
    setup(
        name="spam",
        version="0.1",
        packages=find_packages(),
        # install_requires=[],
        author="Kentaro Wada",
        author_email="www.kentaro.wada@gmail.com",
        # entry_points={"console_scripts": ["spam=spam:main"]},
    )


if __name__ == "__main__":
    main()
