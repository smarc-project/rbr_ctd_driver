from rbr_ctd_driver.parse_rbr_ctd import parse_ctd_data
import pytest
from rbr_ctd_interfaces.msg import RBRCTD

@pytest.mark.parametrize("input_data, expected_output", [
    (
        "2000-01-01 00:04:27.000, 0.0029, 21.7070, 10.2192, 0.0867, 0.0860, 0.0110, 1.0000, 22.0666",
        RBRCTD(
            time_ctd="2000-01-01 00:04:27.000",
            conductivity=0.0029,
            temperature=21.7070,
            pressure=10.2192,
            sea_pressure=0.0867,
            depth=0.0860,
            salinity=0.0110,
            samples=1.0000,
            temperature_conductivity_correction=22.0666
        )
    ),
])
def test_parse_ctd_data(input_data, expected_output):
    """
    Test the parse_ctd_data function with valid input data.
    """
    result = parse_ctd_data(input_data)
    assert result == expected_output

@pytest.mark.parametrize("input_data", [
    ","
])
def test_parse_ctd_data_invalid(input_data):
    """
    Test the parse_ctd_data function with invalid input data.
    """
    with pytest.raises(ValueError):
        parse_ctd_data(input_data)