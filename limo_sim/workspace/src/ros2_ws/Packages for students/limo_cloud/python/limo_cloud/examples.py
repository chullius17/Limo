"""Small examples; no example is active in the default pipeline."""

import numpy as np

from .api import Processor


class BackgroundMask(Processor):
    """Publish a named intermediate for later Python or C++ processors."""

    def process_images(self, frame):
        label = int(self.parameters.get('class_id', 3))
        frame.images['background/mask'] = (frame.labels == label).astype(np.uint8)
        # Distance transforms or other image algorithms can add further arrays
        # here; every later plugin sees the same named intermediate images.


class SelectClasses(Processor):
    """Optional uniform point selection, also provided as a C++ plugin."""

    def process_cloud(self, frame):
        keep = self.parameters.get('keep_classes', [])
        if keep:
            frame.select(np.isin(frame.cloud['class_id'], keep))
