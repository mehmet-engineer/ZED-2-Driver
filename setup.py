from setuptools import setup, find_packages

setup(
    name = "zed_2_driver",
    version = "1.0.0",
    packages = find_packages(where="src"),
    package_dir = {"": "src"},
    
    install_requires = ["zstd", "cython", "numpy", "opencv-python", "pyopengl", "open3d"],
    
    author = "Mehmet KAHRAMAN",
    author_email = "mehmet.kahraman@mcflyrobot.com",
    url = "https://gitlab.mcflyrobot.com/control-software/zed-2-driver",
    
    description = "Stereolabs Zed 2 Stereo Depth Camera Driver in Python.",
    long_description = open('README.md').read(),
    long_description_content_type = 'text/markdown',

    classifiers = [
        'Programming Language :: Python :: 3',
        'Operating System :: Linux',
    ]
)
