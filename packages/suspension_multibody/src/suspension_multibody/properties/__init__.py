"""
Properties files: the numbers, kept out of the template.

A template says which elastic elements exist and which property slots they read.
A properties file says what the numbers are.  The same template loaded with two
files is two different vehicles with one geometry, which is the point: stiffness
and damping are data, not code.

The format is versioned and strict, following the repository's other input
documents (`schema/loader.py`): a root object with `schema_version: 1`, unknown
keys refused, and every failure reported with the file path, the property name,
the field name and the reason.  A properties file that is silently half-read is
worse than one that is rejected, because the difference shows up as a wrong
vehicle rather than as an error.
"""

from .load import (
    ENTRY_KINDS,
    PropertiesDocument,
    PropertiesError,
    PropertySet,
    load_properties,
)

__all__ = [
    "ENTRY_KINDS",
    "PropertiesDocument",
    "PropertiesError",
    "PropertySet",
    "load_properties",
]
