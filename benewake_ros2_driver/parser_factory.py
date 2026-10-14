"""Provide parser selection for supported Benewake lidar models."""

from benewake_ros2_driver.protocol import TFminiSParser


def create_parser(model: str) -> TFminiSParser:
    """
    Create a parser for the selected sensor model.

    Parameters
    ----------
    model : str
        Name of the lidar model.

    Returns
    -------
    TFminiSParser
        Parser instance for the selected model.

    Raises
    ------
    ValueError
        If the requested model is not supported.

    """
    normalized_model = model.strip().lower()

    if normalized_model == 'tfmini_s':
        return TFminiSParser()

    raise ValueError(
        f"Unsupported lidar model '{model}'. "
        'Supported models: tfmini_s'
    )
