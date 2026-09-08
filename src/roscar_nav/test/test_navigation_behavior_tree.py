from pathlib import Path
from xml.etree import ElementTree


def test_global_replanning_does_not_reset_controller_every_second():
    tree = ElementTree.parse(
        Path(__file__).parents[1] / 'behavior_trees' / 'navigate_to_pose_safe.xml'
    )
    rate = tree.find('.//RateController')
    assert rate is not None
    assert float(rate.attrib['hz']) <= 0.2
