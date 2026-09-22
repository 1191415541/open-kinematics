"""
Retired: the PAC2002 scope helpers now live where they belong.

Nothing is left in this module by design -- it is an empty body, not a
forwarding shell, and 08 deletes the file itself.  The three responsibilities it
carried are split like this:

* the *capability read* -- the kernel's own declaration of the USE_MODEs it
  supports and the parameters, feature flags and coefficient families it
  refuses -- is in :mod:`suspension_multibody.kernel.capabilities`, still read
  lazily on first access so that importing the authoring schemas does not open
  the shared library;
* the *schema check* a tire has to pass, ``validate_pac2002_native_scope``, is
  in :mod:`suspension_multibody.schema.pac2002_scope`;
* the *Adams evidence* the comparison manifest publishes next to the
  ``exact_pac2002`` label -- the gated/ungated USE_MODEs, the implemented and
  not-implemented feature lists and the declared-gap registries -- is in
  :mod:`suspension_multibody.adams.pac2002_evidence`.

No production module imports this file any more; if one does again, the
migration regressed.
"""
