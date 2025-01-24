from efficient_ransac import add, substract


def test_add():
    assert add(1, 2) == 3


def test_substract():
    assert substract(1, 2) == -1
