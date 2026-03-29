# Code Quality — Kent Beck's Best Practice Patterns

30 universal principles distilled from Kent Beck's Smalltalk Best Practice Patterns, generalized for any codebase.

1. **Composed Method** — Divide every function into sub-functions that each perform one identifiable task. Keep all operations at the same level of abstraction.
2. **Intention-Revealing Names** — Name methods after what they accomplish, never how.
3. **Replace Comments with Clear Code** — If a comment restates what the code does, delete it. Reserve comments for why, not what.
4. **Constructor Clarity** — Pass all required parameters upfront so callers never receive half-initialized objects.
5. **Single Responsibility** — Each method should have exactly one reason to change.
6. **Say Things Once and Only Once** — Every piece of logic should exist in exactly one place.
7. **Behavior Over State** — Get the public interface right first. Hide internal representation.
8. **Intention-Revealing Function Names** — Name functions after the concept, not the algorithm. `includes(item)` over `linearSearchFor(item)`.
9. **Guard Clauses Over Deep Nesting** — Handle edge cases at the top and return early. Main logic reads without indentation.
10. **Query Methods Return; Commands Mutate** — Separate functions that answer questions from functions that change state.
11. **Explaining Variables** — Assign complex expressions to well-named locals.
12. **Role-Suggesting Names** — Name variables after their role, not their type. `employees` not `employeeList`.
13. **Polymorphism Over Conditionals** — When the same if/switch appears in multiple places, replace with polymorphic objects.
14. **Composition Over Inheritance** — Share implementation via delegation, not subclassing.
15. **Method Object for Complex Logic** — Extract huge methods with many temporaries into their own class.
16. **Execute Around (Resource Bracketing)** — When two actions must always pair (open/close, lock/unlock), expose a single function that accepts a callback.
17. **Explicit Initialization** — Initialize all state at construction time.
18. **Lazy Initialization** — Defer expensive computation to first access when it may not be needed.
19. **Named Constants** — Replace magic literals with named constants. `MAX_RETRIES` over `5`.
20. **Encapsulate Fields** — Access instance fields through getters/setters for a single place to add validation or change notification.
21. **Collection Accessor Safety** — Never return a raw mutable collection. Return a copy, immutable view, or domain-specific methods.
22. **Equality and Hashing Contract** — If you override equality, override hashing to match on the same fields.
23. **Mediating Protocol** — Make collaboration messages between objects explicit and consistently named.
24. **Double Dispatch** — When behavior depends on two types, have the receiver call back the argument with a type-specific method.
25. **Pluggable Behavior Over Subclass Explosion** — Accept a strategy (callback, lambda) instead of creating many subclasses differing in one method.
26. **Collecting Parameter** — Pass a result collection to sub-methods rather than concatenating return values.
27. **Interesting Return Values Only** — Return a value only when the caller needs it. Make return values intentional.
28. **Reversing Method for Readable Flow** — Add convenience methods so all calls flow through one object for left-to-right readability.
29. **Debug Printing** — Override toString/__repr__/inspect for structural debugging info. User-facing display is a separate concern.
30. **Adopt Patterns Incrementally** — Write code, notice friction, then apply the pattern that resolves it. Clean up as you go.
