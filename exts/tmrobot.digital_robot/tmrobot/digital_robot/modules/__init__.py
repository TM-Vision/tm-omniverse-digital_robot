import warnings

with warnings.catch_warnings():
    warnings.filterwarnings(
        "ignore",
        "Protobuf gencode version",
        UserWarning,
        "google.protobuf.runtime_version",
    )
