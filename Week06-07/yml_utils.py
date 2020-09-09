# load the neural network parameters
# no modification needed

import yaml

def load_cfg(cfg_file_path):
    with open(cfg_file_path, 'r') as cfg_file:
        try:
            cfg = yaml.load(cfg_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return cfg
