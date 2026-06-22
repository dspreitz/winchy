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
print("wifi:", w.isconnected(), w.ifconfig()[0] if w.isconnected() else "-")

meta = json.loads(open("mga_chip.json").read())
cc, su = meta["chipcode"], meta["serviceUrl"]
print("chipcode:", cc)

for data in ("uporb_7,ualm", "uporb_1", "uporb_7", "ualm", "uporb_3,ualm,utime"):
    url = "%s?chipcode=%s&gnss=gps&data=%s" % (su, cc, data)
    try:
        r = urequests.get(url)
        body = r.content
        print("[%s] HTTP %d  len=%d" % (data, r.status_code, len(body)))
        if r.status_code != 200 or len(body) < 200:
            try:
                print("   body:", body[:200].decode("utf-8", "replace"))
            except Exception:
                print("   body[:40]:", body[:40])
        r.close()
    except Exception as e:
        print("[%s] ERR %s" % (data, e))
    time.sleep(1)
