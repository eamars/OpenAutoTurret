"""Content identities for the actual local mathematics and native implementation."""
from pathlib import Path
import hashlib
from importlib.metadata import version
from .contracts import digest


def method_identity(native):
    root=Path(__file__).resolve().parent
    files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.glob('*.py'))}
    return {"source_files":files,"native_sha256":hashlib.sha256(Path(native.path).read_bytes()).hexdigest(),
            "dependencies":{name:version(name) for name in ('numpy','scipy')},
            "execution":"OFFLINE_MATHEMATICS","physical_qualification":"NOT_RUN"}


def method_hash(native):
    return digest(method_identity(native))


def identification_component(method):
    # Only the normalized-data fitter and its numerical dependencies determine
    # an identified vector. Changes to acquisition planning or controller scoring
    # require their own revalidation, not silently relabelled bootstrap refits.
    names=('contracts.py','model.py','native.py','identification.py')
    return digest({'source_files':{name:method['source_files'][name] for name in names},
                   'native_sha256':method['native_sha256'],'dependencies':method['dependencies']})
