import io
import numpy as np
from yourdfpy import URDF
import logging

logger = logging.getLogger("transform")


def get_transform(urdf_contents: str, frame_to: str, frame_from: str = None) -> np.ndarray:
    """
    Get the 4x4 transformation matrix between two frames in a URDF.

    Parameters
    ----------
    urdf_contents : str
        The raw URDF contents as a string.
    frame_to : str
        The target frame name.
    frame_from : str, optional
        The source frame name. If None, the base frame of the scene is used.

    Returns
    -------
    np.ndarray
        The 4x4 transformation matrix T such that p_from = T * p_to.
    """
    try:
        # Load the URDF. We use BytesIO since yourdfpy expects bytes for string inputs with encoding headers.
        urdf_bytes = urdf_contents.encode('utf-8') if isinstance(urdf_contents, str) else urdf_contents
        urdf = URDF.load(io.BytesIO(urdf_bytes), build_scene_graph=True, load_meshes=False)

        if frame_from is None:
            frame_from = urdf.scene.graph.base_frame

        T, _ = urdf.scene.graph.get(frame_to, frame_from)
        return T
    except Exception as e:
        logger.error(f"Failed to get transform from '{frame_from}' to '{frame_to}': {e}")
        raise
