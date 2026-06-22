import network, time, json, urequests
from secrets import WIFI_NETWORKS

w = network.WLAN(network.STA_IF)
w.active(True)
if not w.isconnected():
    for ssid, pw in WIFI_NETWORKS:
        w.connect(ssid, pw)
        t = time.time()
        while not w.isconnected() and time.time() - t < 12:
            time.sleep(0.5)
        if w.isconnected():
            break
print("wifi:", w.isconnected())

meta = json.loads(open("mga_chip.json").read())
url = "%s?chipcode=%s&gnss=gps&data=ualm" % (meta["serviceUrl"], meta["chipcode"])
r = urequests.get(url)
print("status:", r.status_code)
h = getattr(r, "headers", None)
print("headers type:", type(h))
if h:
    print("Date:", h.get("Date"))
    try:
        print("keys:", list(h.keys()))
    except Exception as e:
        print("keys err:", e)
r.close()
