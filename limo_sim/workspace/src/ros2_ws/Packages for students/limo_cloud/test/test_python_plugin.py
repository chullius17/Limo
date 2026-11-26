import numpy as np

from limo_cloud import Processor


class OwnershipProbe(Processor):
    def configure(self, parameters):
        self.retained_labels = None
        self.retained_cloud = None

    def process_images(self, frame):
        assert not frame.labels.flags.writeable
        assert frame.labels.strides == (6, 1)
        if self.retained_labels is not None:
            assert self.retained_labels[0, 0] == 1
            assert self.retained_cloud['class_id'].tolist() == [7, 2, 3, 255, 3, 2, 1, 255]
        self.retained_labels = frame.labels
        frame.edit_labels()[0, 0] = 7
        frame.scratch['seen'] = True
        frame.images['test/mask'] = np.array([[1, 0], [0, 1]], dtype=np.uint8)[:, ::-1]

    def process_cloud(self, frame):
        assert frame.scratch['seen']
        assert frame.images['test/mask'].tolist() == [[0, 1], [1, 0]]
        self.retained_cloud = frame.cloud
        frame.select(np.isin(frame.cloud['class_id'], [1, 7]))


class FailureProbe(Processor):
    def configure(self, parameters):
        self.first = True

    def process_cloud(self, frame):
        if self.first:
            self.first = False
            frame.cloud['class_id'][0] = 42
            raise RuntimeError('intentional test failure')


class ExitProbe(Processor):
    def process_cloud(self, frame):
        raise SystemExit('a plugin must not terminate the host interpreter')


class RetainBeforeCpp(Processor):
    def process_cloud(self, frame):
        frame.scratch['before_cpp'] = frame.cloud


class ValidateAfterCpp(Processor):
    def process_cloud(self, frame):
        assert frame.scratch['before_cpp']['class_id'].tolist() == [1, 2, 3, 255, 3, 2, 1, 255]
        assert frame.cloud['class_id'].tolist() == [3, 255, 3, 255]


class SparseProbe(Processor):
    def process_images(self, frame):
        frame.set_samples(np.array([[1, 2], [1, 2], [0, 0]]), np.array([8, 3, 1]))
