# -*- mode: python ; coding: utf-8 -*-

from PyInstaller.utils.hooks import collect_submodules


block_cipher = None

project_root = SPECPATH + "/.."
src_root = project_root + "/src"

hiddenimports = [
    "matplotlib.backends.backend_tkagg",
    "mpl_toolkits.mplot3d",
]
hiddenimports += collect_submodules("kinematics.suspensions")

excluded_modules = [
    "IPython",
    "PIL.AvifImagePlugin",
    "PIL.ImageQt",
    "PIL.Jpeg2KImagePlugin",
    "PIL._avif",
    "PIL._imagingcms",
    "PIL._webp",
    "PyQt5",
    "PyQt6",
    "PySide2",
    "PySide6",
    "click",
    "jupyter",
    "notebook",
    "pandas",
    "pyarrow",
    "pytest",
    "rich",
    "sympy",
    "typer",
    "wx",
    "matplotlib.backends.backend_gtk3agg",
    "matplotlib.backends.backend_gtk4agg",
    "matplotlib.backends.backend_macosx",
    "matplotlib.backends.backend_qt5agg",
    "matplotlib.backends.backend_qtagg",
    "matplotlib.backends.backend_wxagg",
]

a = Analysis(
    [src_root + "/kinematics/gui/app.py"],
    pathex=[src_root],
    binaries=[],
    datas=[],
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={
        "matplotlib": {
            "backends": ["TkAgg"],
        },
    },
    runtime_hooks=[],
    excludes=excluded_modules,
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="KinematicsWorkbench",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="KinematicsWorkbench",
)
