"""Tests for selecting lidar protocol parsers."""


from benewake_ros2_driver.parser_factory import create_parser
from benewake_ros2_driver.protocol import TFminiSParser
import pytest


def test_create_tfmini_s_parser():
    """Create the parser for TFmini-S."""
    parser = create_parser('tfmini_s')
    assert isinstance(parser, TFminiSParser)


def test_model_name_is_case_insensitive():
    """Accept uppercase and surrounding whitespace."""
    parser = create_parser(' TFMINI_S ')
    assert isinstance(parser, TFminiSParser)


def test_unknown_model_raises_value_error():
    """Reject models that do not have an implemented parser."""
    with pytest.raises(ValueError, match='Unsupported lidar model'):
        create_parser('tfmini_plus')
