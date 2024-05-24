from math import sin, cos, tan, radians, asin, acos, pi, degrees

def get_sun_elevation_angle(geographical_latitude: float, sun_declination: float, hour_angle: float) -> float:
    """
    Returns the sun elevation angle in rad (arm rotation).
    
    :param geographical_latitude: Geographical latitude of the assumed location in rad
    :param sun_declination: (Seasonal) sun declination of the earth in rad
    :param hour_angle: (Daily) hour angle of the earth in rad
    :return: sun elevation angle in rad
    """
    # TODO: Separate calculations
    return asin(cos(geographical_latitude) * cos(sun_declination) * cos(hour_angle) + sin(geographical_latitude) * sin(sun_declination))

def get_sun_azimuth_angle(sun_elevation_angle: float, geographical_latitude: float, sun_declination: float, true_solar_time: float) -> float:
    """
    Returns the sun azimuth angle in rad (table rotation) where
    - North sun azimuth angle = 0 rad
    - East sun azimuth angle = pi/2 rad
    - South sun azimuth angle = pi rad
    - West sun azimuth angle = 3pi/2 rad
    
    :param sun_elevation_angle: Sun elevation angle in rad
    :param geographical_latitude: (Seasonal) sun declination of the earth in rad
    :param sun_declination: (Seasonal) sun declination of the earth in rad
    :param true_solar_time: True Solar Time in (float) hours
    :return: sun azimuth angle in rad
    """
    # TODO: Separate calculations
    common = (sin(sun_elevation_angle) * sin(geographical_latitude) - sin(sun_declination)) / (cos(sun_elevation_angle) * cos(geographical_latitude))
    if true_solar_time <= 12.00:
        return pi - acos(common)
    else: # true_solar_time <= 12.00:
        return pi + acos(common)

def get_sun_declination(day: float) -> float:
    """
    Returns an approximation of the sun declination in rad using Fourier series.
    
    :param day: day of the year
    :return: sun declination in rad
    """
    rad_day = get_rad_day(day)
    
    return radians(0.3948 - 23.2559 * cos(rad_day + radians(9.1)) - 0.3915 * cos(2 * rad_day + radians(5.4)) - 0.1764 * cos(3 * rad_day + radians(26)))

def get_hour_angle(true_solar_time: float) -> float:
    """
    Returns the hour angle in rad.
    
    :param true_solar_time: True Solar Time in (float) hours
    :return: hour angle in rad
    """
    common = radians(15) * abs(12 - true_solar_time)
    if true_solar_time < 12.00:
        return -common
    elif true_solar_time > 12.00:
        return common
    else:
        # Implied behaviour
        return 0
    
def get_true_solar_time(local_clock_time: float, geographical_longitude: float, equation_of_time: float, standard_time_meridian_longitude: float = pi / 12) -> float:
    """
    Returns the true solar time in (float) hours.
    
    :param local_clock_time: Local clock Time in (float) hours
    :param geographical_longitude: geographical longitude in (float) hours
    :param standard_time_meridian_longitude: Longitude of the standard time meridian (pi/12 rad)
    """
    return local_clock_time + 12 * (geographical_longitude - standard_time_meridian_longitude) / pi + equation_of_time

def get_equation_of_time(day: float) -> float:
    rad_day = get_rad_day(day)
    minutes = 0.0066 + 7.3525 * cos(rad_day + radians(85.9)) + 9.9359 * cos(2 * rad_day + radians(108.9)) + 0.3387 * cos(3 * rad_day + radians(105.2))
    return minutes / 60

def get_rad_day(day: float) -> float:
    """
    Returns the relative rotation of the earth from the start of the year in rad/day.
    
    :param day: day of the year
    :return: relative degrees of the day in rad of the day
    """
    return (2 * pi * day) / 365

def get_time_sunrise(geographical_latitude: float, sun_declination: float) -> float:
    """
    Returns the time of a sunrise in (float) hours.
    
    :param geographical_latitude: Geographical latitude of the assumed location in rad
    :param sun_declination: (Seasonal) sun declination of the earth in rad
    :return: sunrise time in (float hours)
    """
    return 12 - 12 * (acos(-tan(geographical_latitude) * tan(sun_declination))) / pi

def get_time_sunset(geographical_latitude: float, sun_declination: float) -> float:
    """
    Returns the time of a sunset in (float) hours.
    
    :param geographical_latitude: Geographical latitude of the assumed location in rad
    :param sun_declination: (Seasonal) sun declination of the earth in rad
    :return: sunset time in (float hours)
    """
    return 12 + 12 * (acos(-tan(geographical_latitude) * tan(sun_declination))) / pi

def get_daylight_duration(geographical_latitude: float, sun_declination: float) -> float:
    """
    Returns the duration of daylight in (float) hours.
    
    :param geographical_latitude: Geographical latitude of the assumed location in rad
    :param sun_declination: (Seasonal) sun declination of the earth in rad
    :return: daylight duration in (float hours)
    """
    return 2 * (acos(-tan(geographical_latitude) * tan(sun_declination))) / 15