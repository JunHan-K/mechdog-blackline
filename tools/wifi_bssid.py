"""Connect Windows Wi-Fi to ONE specific access point (BSSID) when several share an SSID
(e.g. every team's camera is called HW_ESP32S3CAM). Uses wlanapi WlanConnect with a BSSID list;
needs an existing Windows profile for the SSID (connect to it once normally).

    py tools\\wifi_bssid.py                       # list APs of HW_ESP32S3CAM + current BSSID
    py tools\\wifi_bssid.py 8c:bf:ea:87:f1:29     # connect to that camera
"""
import ctypes
import ctypes.wintypes as wt
import re
import subprocess
import sys
import time

CAM_SSID = 'HW_ESP32S3CAM'


class GUID(ctypes.Structure):
    _fields_ = [('Data1', wt.DWORD), ('Data2', wt.WORD), ('Data3', wt.WORD), ('Data4', ctypes.c_ubyte * 8)]


class WLAN_INTERFACE_INFO(ctypes.Structure):
    _fields_ = [('InterfaceGuid', GUID), ('strInterfaceDescription', wt.WCHAR * 256), ('isState', ctypes.c_uint)]


class WLAN_INTERFACE_INFO_LIST(ctypes.Structure):
    _fields_ = [('dwNumberOfItems', wt.DWORD), ('dwIndex', wt.DWORD), ('InterfaceInfo', WLAN_INTERFACE_INFO * 1)]


class DOT11_SSID(ctypes.Structure):
    _fields_ = [('uSSIDLength', wt.ULONG), ('ucSSID', ctypes.c_ubyte * 32)]


class NDIS_OBJECT_HEADER(ctypes.Structure):
    _fields_ = [('Type', ctypes.c_ubyte), ('Revision', ctypes.c_ubyte), ('Size', wt.USHORT)]


class DOT11_BSSID_LIST(ctypes.Structure):
    _fields_ = [('Header', NDIS_OBJECT_HEADER), ('uNumOfEntries', wt.ULONG),
                ('uTotalNumOfEntries', wt.ULONG), ('BSSIDs', (ctypes.c_ubyte * 6) * 1)]


class WLAN_CONNECTION_PARAMETERS(ctypes.Structure):
    _fields_ = [('wlanConnectionMode', ctypes.c_uint), ('strProfile', wt.LPCWSTR),
                ('pDot11Ssid', ctypes.POINTER(DOT11_SSID)), ('pDesiredBssidList', ctypes.POINTER(DOT11_BSSID_LIST)),
                ('dot11BssType', ctypes.c_uint), ('dwFlags', wt.DWORD)]


def _netsh(*a):
    return subprocess.run(['netsh', 'wlan', *a], capture_output=True, text=True, errors='replace').stdout


def norm(b):
    return b.strip().lower().replace('-', ':')


def current():
    """(ssid, bssid) of the current connection."""
    ssid = bssid = None
    for line in _netsh('show', 'interfaces').splitlines():
        k, _, v = line.partition(':')
        k = k.strip()
        if k == 'SSID':
            ssid = v.strip()
        elif 'BSSID' in k:
            bssid = norm(line.split(':', 1)[1])
    return ssid, bssid


def list_bssids(ssid=CAM_SSID):
    """[(bssid, signal%)] for an SSID from the last scan."""
    out, cur, res = _netsh('show', 'networks', 'mode=bssid'), None, []
    for line in out.splitlines():
        s = line.strip()
        m = re.match(r'^SSID \d+ : (.*)$', s)
        if m:
            cur = m.group(1).strip()
            continue
        m = re.match(r'^BSSID \d+\s*: (.*)$', s)
        if m and cur == ssid:
            res.append([norm(m.group(1)), None])
        m = re.match(r'^Signal\s*: (\d+)%', s)
        if m and cur == ssid and res and res[-1][1] is None:
            res[-1][1] = int(m.group(1))
    return [tuple(r) for r in res]


def connect(bssid, ssid=CAM_SSID, profile=None, timeout=15):
    """Connect to ssid on the given AP. Returns True when `netsh` reports that BSSID."""
    bssid = norm(bssid)
    api = ctypes.WinDLL('wlanapi.dll')
    h, ver = wt.HANDLE(), wt.DWORD()
    if api.WlanOpenHandle(2, None, ctypes.byref(ver), ctypes.byref(h)):
        raise OSError('WlanOpenHandle failed')
    try:
        plist = ctypes.POINTER(WLAN_INTERFACE_INFO_LIST)()
        if api.WlanEnumInterfaces(h, None, ctypes.byref(plist)) or not plist.contents.dwNumberOfItems:
            raise OSError('no Wi-Fi interface')
        guid = plist.contents.InterfaceInfo[0].InterfaceGuid
        if current()[0] == ssid and current()[1] != bssid:
            api.WlanDisconnect(h, ctypes.byref(guid), None)
            time.sleep(1.0)
        sb = ssid.encode()
        dssid = DOT11_SSID(len(sb), (ctypes.c_ubyte * 32)(*sb))
        bl = DOT11_BSSID_LIST()
        bl.Header = NDIS_OBJECT_HEADER(0x80, 1, ctypes.sizeof(DOT11_BSSID_LIST))
        bl.uNumOfEntries = bl.uTotalNumOfEntries = 1
        bl.BSSIDs[0] = (ctypes.c_ubyte * 6)(*bytes.fromhex(bssid.replace(':', '')))
        p = WLAN_CONNECTION_PARAMETERS(0, profile or ssid, ctypes.pointer(dssid), ctypes.pointer(bl), 1, 0)
        rc = api.WlanConnect(h, ctypes.byref(guid), ctypes.byref(p), None)
        if rc:
            raise OSError('WlanConnect error %d (Windows profile for %s missing?)' % (rc, ssid))
    finally:
        api.WlanCloseHandle(h, None)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if current() == (ssid, bssid):
            return True
        time.sleep(0.5)
    return False


if __name__ == '__main__':
    if len(sys.argv) > 1:
        print('connected' if connect(sys.argv[1]) else 'FAILED', current())
    else:
        print('current:', current())
        for b, s in list_bssids():
            print(b, s, '%')
