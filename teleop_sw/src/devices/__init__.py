
import warnings
DEVICES_DICT = {}

try:
    from .child import Child
    DEVICES_DICT['child'] = Child
except Exception as e:
    warnings.warn('Child not found, or some error happened while importing Child')
    warnings.warn("error:" + str(e))


print("Available devices: ", DEVICES_DICT.keys())
