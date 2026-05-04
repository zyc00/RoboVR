# Publishing RoboVR

RoboVR's PyPI package only contains the `robovr` Python package. The Quest APK,
native C++ sources, Android build artifacts, OpenXR external source tree, local
build directories, APKs, and generated shared libraries are not part of the
wheel.

## Build And Check

```bash
python -m pip install build twine
rm -rf dist build *.egg-info
python -m build
twine check dist/*
python -m pytest
```

Verify the wheel contents:

```bash
python - <<'PY'
import glob
import zipfile

wheel = glob.glob("dist/*.whl")[0]
with zipfile.ZipFile(wheel) as z:
    names = z.namelist()
    assert any(n.startswith("robovr/quest3/") for n in names)
    assert not any(
        ".gradle/" in n
        or n.startswith("external/")
        or n.startswith("quest_client/app/build/")
        for n in names
    )
    print("wheel ok", wheel)
PY
```

## Upload

```bash
twine upload --repository testpypi dist/*
twine upload dist/*
```

If the `robovr` name is unavailable on PyPI, rename the package, for example to
`robo-vr` or `robovr-quest`, then update RoboInfra's optional dependency from
`robovr>=0.1.0` to the final published package name.
