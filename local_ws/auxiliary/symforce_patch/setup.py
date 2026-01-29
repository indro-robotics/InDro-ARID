# ----------------------------------------------------------------------------
# SymForce - Copyright 2022, Skydio, Inc.
# This source code is under the Apache 2.0 license found in the LICENSE file.
# ----------------------------------------------------------------------------

import multiprocessing
import os
import re
import subprocess
import sys
import typing as T
from pathlib import Path

from setuptools import Extension
from setuptools import find_namespace_packages
from setuptools import find_packages
from setuptools import setup
from setuptools.command.build_ext import build_ext
from setuptools.command.develop import develop
from setuptools.command.install import install

SOURCE_DIR = Path(__file__).resolve().parent
ESCAPED_SOURCE_DIR = Path(str(SOURCE_DIR).replace(" ", "%20"))


class CMakeExtension(Extension):
    """
    CMake extension type.
    """

    def __init__(self, name: str):
        Extension.__init__(self, name, sources=[])


class PatchDevelop(develop):
    """
    develop is the legacy command (pre setuptools==64.0.0, which implemented
    the pep 660 hook build_editable) to build a package in editable mode.
    """

    def run(self) -> None:  # type: ignore[override]
        self.distribution.get_command_obj("build_ext").editable_mode = True  # type: ignore[misc]
        super().run()


class CMakeBuild(build_ext):
    """
    Custom extension builder that runs CMake.
    """

    def run(self) -> None:
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError as ex:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: "
                + ", ".join(e.name for e in self.extensions)
            ) from ex

        build_temp_path = Path(self.build_temp)
        build_directory = build_temp_path.resolve()

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={build_directory}",
            "-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-rpath,'$ORIGIN/../..'",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
        ]

        editable_mode = self.editable_mode if hasattr(self, "editable_mode") else False

        if editable_mode:
            if sys.platform == "linux" or sys.platform == "linux2":
                cmake_args.append("-DCMAKE_BUILD_RPATH=$ORIGIN")
            elif sys.platform == "darwin":
                cmake_args.append("-DCMAKE_BUILD_RPATH=@loader_path")

        cfg = "Debug" if self.debug else "Release"
        build_args = ["--config", cfg]

        cmake_args += [
            f"-DCMAKE_BUILD_TYPE={cfg}",
            "-DSYMFORCE_BUILD_TESTS=OFF",
            "-DSYMFORCE_BUILD_EXAMPLES=OFF",
        ]

        build_args += ["--", f"-j{multiprocessing.cpu_count()}"]

        self.build_args = build_args

        env = os.environ.copy()
        env["CXXFLAGS"] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get("CXXFLAGS", ""), self.distribution.get_version()
        )

        build_temp_path.mkdir(parents=True, exist_ok=True)

        print("-" * 10, "Running CMake prepare", "-" * 40)
        subprocess.run(
            ["cmake", str(SOURCE_DIR)] + cmake_args, cwd=self.build_temp, env=env, check=True
        )

        print("-" * 10, "Building extensions", "-" * 40)
        cmake_cmd = ["cmake", "--build", "."] + self.build_args
        subprocess.run(cmake_cmd, cwd=self.build_temp, check=True)

        if editable_mode:
            symengine_wrapper = maybe_find_symengine_wrapper(
                build_temp_path, self.get_ext_filename("symengine_wrapper")
            )
            if symengine_wrapper:
                self.copy_file(
                    str(symengine_wrapper),
                    str(
                        SOURCE_DIR
                        / "third_party"
                        / "symenginepy"
                        / "symengine"
                        / "lib"
                        / self.get_ext_filename("symengine_wrapper")
                    ),
                )

            for cc_sym_dependency in build_temp_path.glob("libsymforce_*"):
                self.copy_file(
                    str(cc_sym_dependency),
                    str(SOURCE_DIR / cc_sym_dependency.name),
                )

        for ext in self.extensions:
            self.move_output(ext)

    def move_output(self, ext: CMakeExtension) -> None:
        if ext.name == "lcmtypes":
            build_temp_path = Path(self.build_temp)
            dest_path_dir = Path(self.get_ext_fullpath(ext.name)).resolve().parent
            if self.inplace:
                dest_path = dest_path_dir / "lcmtypes_build" / "lcmtypes"
            else:
                dest_path = dest_path_dir / "lcmtypes"

            self.copy_tree(
                str(build_temp_path / "lcmtypes" / "python2.7" / "lcmtypes"),
                str(dest_path),
            )
            return

        build_temp = Path(self.build_temp).resolve()
        extension_source_paths = {
            "cc_sym": build_temp / "pybind" / self.get_ext_filename("cc_sym")
        }

        dest_path = Path(self.get_ext_fullpath(ext.name)).resolve()
        dest_directory = dest_path.parents[0]
        dest_directory.mkdir(parents=True, exist_ok=True)
        self.copy_file(extension_source_paths[ext.name], str(dest_path))


def maybe_rewrite_local_dependencies(dep_list: T.List[str]) -> T.List[str]:
    if "SYMFORCE_REWRITE_LOCAL_DEPENDENCIES" in os.environ:

        def filter_local(s: str) -> str:
            if "@" in s:
                s = f"{s.split('@')[0]}=={os.environ['SYMFORCE_REWRITE_LOCAL_DEPENDENCIES']}"
            return s

        return [filter_local(dependency) for dependency in dep_list]
    else:
        return dep_list


def maybe_find_symengine_wrapper(build_dir: Path, ext_filename: str) -> T.Optional[Path]:
    symengine_wrapper_candidates = list(
        build_dir.glob(
            f"symengine_install/**/lib/python{sys.version_info.major}.{sys.version_info.minor}/*-packages/symengine/lib/{ext_filename}"
        )
    )

    if len(symengine_wrapper_candidates) > 1:
        raise FileNotFoundError(
            f"Expected to find exactly one symengine_wrapper.so, but found {len(symengine_wrapper_candidates)}: {symengine_wrapper_candidates}"
        )

    return next(iter(symengine_wrapper_candidates), None)


def find_symengine_wrapper(build_dir: Path, ext_filename: str) -> Path:
    symengine_wrapper = maybe_find_symengine_wrapper(build_dir, ext_filename)
    if symengine_wrapper is None:
        raise FileNotFoundError(f"Could not find symengine_wrapper.so in {build_dir}")
    return symengine_wrapper


class InstallWithExtras(install):
    """
    Custom install step that:
        1) Installs symenginepy so it can be imported
        2) Installs additional shared libraries needed by cc_sym (e.g. libmetis)
        3) Installs lcmtypes python package
    """

    def run(self) -> None:
        super().run()

        build_ext_obj = self.distribution.get_command_obj("build_ext")
        assert isinstance(build_ext_obj, CMakeBuild)
        build_dir = Path(build_ext_obj.build_temp)

        # 1) Install symengine_wrapper.so into installed platlib
        symengine_wrapper = find_symengine_wrapper(
            build_dir, build_ext_obj.get_ext_filename("symengine_wrapper")
        )
        self.copy_file(
            str(symengine_wrapper),
            Path.cwd()
            / self.install_platlib
            / "symengine"
            / "lib"
            / build_ext_obj.get_ext_filename("symengine_wrapper"),
        )

        # 2) Install libsymforce_* next to the installed cc_sym extension
        cc_sym_dest = Path(build_ext_obj.get_ext_fullpath("cc_sym")).resolve()
        cc_sym_dir = cc_sym_dest.parent
        cc_sym_dir.mkdir(parents=True, exist_ok=True)

        for cc_sym_dependency in build_dir.glob("libsymforce_*"):
            self.copy_file(
                str(cc_sym_dependency),
                str(cc_sym_dir / cc_sym_dependency.name),
            )

        # NOTE: we deliberately do NOT run "cmake --build . --target install" here.


setup_requirements = [
    "setuptools>=62.3.0",  # For package data globs
    "setuptools-scm>=8",
    "wheel",
    "pip",
    "cmake>=3.17,<3.27",
    "cython>=0.19.1,<3",
    f"skymarshal @ file://localhost/{ESCAPED_SOURCE_DIR}/third_party/skymarshal",
]

docs_requirements = [
    "furo",
    "ipykernel",
    "ipython-genutils",
    "matplotlib",
    "myst-parser",
    "nbsphinx",
    "nbstripout",
    "pandas",
    "plotly",
    "Sphinx",
    "sphinx-copybutton",
    "sphinxext-opengraph",
    "breathe",
]


def symforce_rev() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], check=True, text=True, stdout=subprocess.PIPE
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return "main"


def fixed_readme() -> str:
    readme = Path("README.md").read_text(encoding="UTF8")

    readme = readme.replace(
        "docs/static/images/",
        f"https://raw.githubusercontent.com/symforce-org/symforce/{symforce_rev()}/docs/static/images/",
    )

    readme = re.sub(
        r"<!--\s*DARK_MODE_ONLY\s*-->((?!DARK_MODE_ONLY).)*<!--\s*/DARK_MODE_ONLY\s*-->",
        "",
        readme,
        flags=re.MULTILINE | re.DOTALL,
    )

    return readme


if __name__ == "__main__":
    setup(
        long_description=fixed_readme(),
        long_description_content_type="text/markdown",
        packages=find_namespace_packages(where=".", include=["symforce*"])
        + find_packages(where=".", exclude=["symforce*"])
        + find_packages(where="third_party/symenginepy"),
        package_dir={
            "symforce": "symforce",
            "symengine": "third_party/symenginepy/symengine",
            "lcmtypes": "lcmtypes_build/lcmtypes",
        },
        package_data={
            "": ["*.jinja", "*.mtx", "README*", ".clang-format", "py.typed", "ruff.toml"]
        },
        url="https://symforce.org",
        cmdclass=dict(
            build_ext=CMakeBuild,
            install=InstallWithExtras,
            develop=PatchDevelop,
        ),
        ext_modules=[CMakeExtension("cc_sym"), CMakeExtension("lcmtypes")],
        install_requires=maybe_rewrite_local_dependencies(
            [
                "ruff",
                "clang-format",
                "graphviz",
                "jinja2",
                "numpy<2.0",
                "scipy",
                f"skymarshal @ file://localhost/{ESCAPED_SOURCE_DIR}/third_party/skymarshal",
                "sympy~=1.11.1",
                f"symforce-sym @ file://localhost/{ESCAPED_SOURCE_DIR}/gen/python",
                "typing-extensions; python_version<'3.9'",
            ]
        ),
        setup_requires=setup_requirements,
        extras_require={},
    )
