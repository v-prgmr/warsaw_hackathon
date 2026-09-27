"""Turn typed / spoken text into a search command (no ROS).

    "search for a red cup" / "find the red cup" / "where is my bottle?" / "red cup"
        -> ("search", "red cup") ...
    "stop" / "cancel"   -> ("stop", None)       stop searching
    "clear"             -> ("clear", None)      remove the boxes of found objects
"""
import re

_STOP = {"stop", "cancel", "stop search", "stop searching"}
_CLEAR = {"clear", "clear all", "clear boxes", "reset"}
_PREFIXES = ("search for", "search", "find me", "find", "look for", "looking for", "locate",
             "where is", "where's", "where are", "show me")
_ARTICLES = ("a ", "an ", "the ", "my ", "some ")


def parse_command(text):
    t = re.sub(r"\s+", " ", str(text or "").strip().lower()).strip(" .!?")
    if not t:
        return None, None
    if t in _STOP:
        return "stop", None
    if t in _CLEAR:
        return "clear", None
    for prefix in _PREFIXES:
        if t.startswith(prefix + " "):
            t = t[len(prefix) + 1:]
            break
    for article in _ARTICLES:
        if t.startswith(article):
            t = t[len(article):]
            break
    t = t.strip(" .!?")
    return ("search", t) if t else (None, None)
