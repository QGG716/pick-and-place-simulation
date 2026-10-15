"""Portable asset identities, local mounts and ROS resource URIs are distinct."""
from pathlib import Path, PurePosixPath, PureWindowsPath
from urllib.parse import quote


def asset_relative(value, *, migrate_windows=False):
    if not isinstance(value, str) or not value or '\x00' in value:
        raise ValueError('INVALID_ASSET_PATH')
    if '\\' in value:
        if not migrate_windows: raise ValueError('NONPORTABLE_ASSET_PATH:'+value)
        value = value.replace('\\', '/')
    if (PureWindowsPath(value).drive or value.startswith('/') or ':' in value or
            any(p in ('', '.', '..') for p in value.split('/'))):
        raise ValueError('ASSET_PATH_MUST_BE_RELATIVE:'+value)
    return PurePosixPath(value).as_posix()


def resolve_asset(root, relative):
    root = Path(root).resolve()
    path = root.joinpath(*PurePosixPath(asset_relative(relative)).parts).resolve()
    path.relative_to(root)  # Also reject symlinks escaping the mounted repository.
    return path


def resource_uri(root, relative, native_asset_root=None):
    relative = asset_relative(relative)
    if native_asset_root is None:
        return resolve_asset(root, relative).as_uri()
    # This is a rootfs POSIX mount, regardless of the generator's host OS.
    mount = str(native_asset_root)
    if not mount.startswith('/') or '\\' in mount or ':' in mount or '..' in mount.split('/'):
        raise ValueError('NATIVE_ASSET_ROOT_MUST_BE_ABSOLUTE_POSIX')
    return 'file://' + quote(str(PurePosixPath(mount)/relative), safe='/')
