import time
import math
import board
import busio
import digitalio
import storage
import adafruit_sdcard
import adafruit_bme280
import adafruit_bno055
import adafruit_gps

PERIOD = 1.0
BURST_PERIOD = 0.01
BURST_DURATION = 2.0
IMPACT_THRESHOLD_MS2 = 15.0
INIT_RETRIES = 5
INIT_RETRY_DELAY = 0.5

HEADER = (
    "num,time_s,mode,T_C,P_hPa,humidity_pct,alt_rel_m,"
    "Bx_uT,By_uT,Bz_uT,B_uT,"
    "ax,ay,az,"
    "lat,lon,alt_gps_m,satellites,fix,"
    "cal_sys_gyr_acc_mag"
)

i2c = busio.I2C(board.GP5, board.GP4)
uart = busio.UART(board.GP0, board.GP1, baudrate=9600, timeout=0)
spi = busio.SPI(board.GP10, board.GP11, board.GP12)
sd_cs = digitalio.DigitalInOut(board.GP13)


def init_with_retry(factory, retries=INIT_RETRIES, delay=INIT_RETRY_DELAY):
    last_error = None
    for _ in range(retries):
        try:
            return factory()
        except Exception as error:
            last_error = error
            time.sleep(delay)
    raise RuntimeError("sensor init failed: {}".format(last_error))


def init_bme280():
    def factory():
        for address in (0x76, 0x77):
            try:
                return adafruit_bme280.Adafruit_BME280_I2C(i2c, address=address)
            except ValueError:
                pass
        raise RuntimeError("BME280 not found on I2C")
    return init_with_retry(factory)


def init_bno055():
    return init_with_retry(lambda: adafruit_bno055.BNO055_I2C(i2c))


def init_log_file():
    try:
        sd = adafruit_sdcard.SDCard(spi, sd_cs)
        vfs = storage.VfsFat(sd)
        storage.mount(vfs, "/sd")
        return open("/sd/caliban_log.csv", "a")
    except Exception:
        return None


def reference_pressure(sensor, samples=20):
    total = 0.0
    for _ in range(samples):
        total += sensor.pressure
        time.sleep(0.1)
    return total / samples


def fmt(x, decimals):
    if x is None:
        return ""
    return ("%." + str(decimals) + "f") % x


def magnitude(v):
    if v is None or None in v:
        return None
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def read_acceleration(sensor):
    try:
        return sensor.acceleration
    except Exception:
        return (None, None, None)


def read_mag_cal(sensor):
    try:
        mag = sensor.magnetic
        cal = sensor.calibration_status
    except Exception:
        mag = cal = None
    if mag is None:
        mag = (None, None, None)
    return mag, cal


def read_atmosphere(sensor):
    try:
        return sensor.temperature, sensor.pressure, sensor.humidity, sensor.altitude
    except Exception:
        return None, None, None, None


def read_gps(sensor):
    sensor.update()
    has_fix = sensor.has_fix
    lat = sensor.latitude if has_fix else None
    lon = sensor.longitude if has_fix else None
    alt_gps = sensor.altitude_m if has_fix else None
    satellites = sensor.satellites
    return lat, lon, alt_gps, satellites, has_fix


def build_line(num, elapsed, mode, temperature, pressure, humidity, altitude,
               mag, acc, lat, lon, alt_gps, satellites, has_fix, cal):
    return ",".join((
        str(num),
        fmt(elapsed, 2),
        mode,
        fmt(temperature, 2),
        fmt(pressure, 2),
        fmt(humidity, 2),
        fmt(altitude, 1),
        fmt(mag[0], 2),
        fmt(mag[1], 2),
        fmt(mag[2], 2),
        fmt(magnitude(mag), 2),
        fmt(acc[0], 2),
        fmt(acc[1], 2),
        fmt(acc[2], 2),
        fmt(lat, 6),
        fmt(lon, 6),
        fmt(alt_gps, 1),
        "" if satellites is None else str(satellites),
        "1" if has_fix else "0",
        "" if cal is None else "".join(str(c) for c in cal),
    ))


def write_line(line, log_file):
    print(line)
    if log_file is not None:
        log_file.write(line + "\n")
        log_file.flush()


bme = init_bme280()
bme.sea_level_pressure = reference_pressure(bme)
bno = init_bno055()
gps = adafruit_gps.GPS(uart, debug=False)
log_file = init_log_file()

write_line(HEADER, log_file)

t0 = time.monotonic()
last_sample = t0
num = 0
burst_until = None
was_in_burst = False
burst_buffer = []
last_atmosphere = (None, None, None, None)
last_gps = (None, None, None, None, False)

while True:
    now = time.monotonic()
    acc = read_acceleration(bno)
    accel_mag = magnitude(acc)

    if burst_until is None and accel_mag is not None and accel_mag > IMPACT_THRESHOLD_MS2:
        burst_until = now + BURST_DURATION

    in_burst = burst_until is not None and now < burst_until
    if burst_until is not None and now >= burst_until:
        burst_until = None
        in_burst = False

    if was_in_burst and not in_burst:
        for buffered_line in burst_buffer:
            write_line(buffered_line, log_file)
        burst_buffer = []
    was_in_burst = in_burst

    period = BURST_PERIOD if in_burst else PERIOD
    if now - last_sample < period:
        continue
    last_sample = now
    num += 1

    if in_burst:
        mag, cal = (None, None, None), None
        temperature, pressure, humidity, altitude = last_atmosphere
        lat, lon, alt_gps, satellites, has_fix = last_gps
        line = build_line(num, now - t0, "impact", temperature, pressure,
                           humidity, altitude, mag, acc, lat, lon, alt_gps,
                           satellites, has_fix, cal)
        burst_buffer.append(line)
    else:
        mag, cal = read_mag_cal(bno)
        temperature, pressure, humidity, altitude = read_atmosphere(bme)
        last_atmosphere = (temperature, pressure, humidity, altitude)
        lat, lon, alt_gps, satellites, has_fix = read_gps(gps)
        last_gps = (lat, lon, alt_gps, satellites, has_fix)
        line = build_line(num, now - t0, "normal", temperature, pressure,
                           humidity, altitude, mag, acc, lat, lon, alt_gps,
                           satellites, has_fix, cal)
        write_line(line, log_file)
