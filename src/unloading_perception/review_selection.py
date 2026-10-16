"""Identity-based exclusions for the current review, leaving archives untouched."""


def validate_review_capture(metadata, module, policy):
    for excluded in policy['excluded_capture_groups']:
        if (metadata['sensor_epoch'] == excluded['sensor_epoch']
                and metadata['frame_sequence'] == excluded['frame_sequence']
                and module in excluded['modules']):
            raise ValueError('EXCLUDED_CAPTURE_IDENTITY:' + metadata['capture_id'])
    return metadata['capture_id']
