import 'dart:collection';

/// A simple LIFO stack backed by a [ListQueue].
class Stack<T> {
  final _list = ListQueue<T>();

  /// Whether the stack is empty.
  bool get isEmpty => _list.isEmpty;

  /// Whether the stack is not empty.
  bool get isNotEmpty => _list.isNotEmpty;

  /// Pushes [e] onto the top of the stack.
  void push(T e) => _list.addLast(e);

  /// Removes and returns the top element.
  T pop() {
    final res = _list.last;
    _list.removeLast();
    return res;
  }

  /// Returns the top element without removing it.
  T top() => _list.last;

  /// The number of elements in the stack.
  int get length => _list.length;

  /// Whether the stack contains [x].
  bool contains(T x) => _list.contains(x);
}
