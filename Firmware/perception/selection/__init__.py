"""Perception subpackage: who the operator has chosen (§27–§35).

``protocol.py`` holds the request/ack messages, ``policy.py`` the selection policies,
``alias_map.py`` §25.1's merge aliases, and ``target_selection_manager.py`` the
authoritative selection state. No module here may import a motor backend (§49), and none
of them may command anything: the output is a selection *state* and a
``SelectedTargetObservation``.
"""
