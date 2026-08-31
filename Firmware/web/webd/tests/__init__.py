"""Tests for the webd web service.

These tests exercise the webd (client side of the web socket) and its FastAPI
app WITHOUT a real controld, CAN, or camera. A :class:`FakeControld` provides
the server side of the socket, so the whole webd path is exercised in-process.

Run: ``python3 -m unittest web.webd.tests -v``  (or the specific test module).
"""
