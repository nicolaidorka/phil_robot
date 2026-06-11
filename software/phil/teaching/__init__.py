"""Teaching: the taught-well table and the interactive jog/teach console.

Only ``TeachTable`` is re-exported. Do NOT import ``jog_teach`` here: it imports
``phil.robot``, which imports ``teaching.teach`` -- importing jog_teach at package
init time would create an import cycle. Reach it as ``phil.teaching.jog_teach``.
"""
from .teach import TeachTable

__all__ = ["TeachTable"]
