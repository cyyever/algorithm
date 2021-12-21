import cyy_algorithm


def test_heap():
    heap = cyy_algorithm.Heap()
    heap.insert("a", 1)
    top = heap.top_key()
    assert top == "a"
    heap.pop()
