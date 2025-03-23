///
///  _____          __  __ ______ _   _
/// |  __ `   /`   |  `/  |  ____| ` | |
/// | |__) | /  `  | `  / | |__  |  `| |
/// |  _  / / /` ` | |`/| |  __| | . ` |
/// | | ` `/ ____ `| |  | | |____| |`  |
/// |_|  `_`/    `_`_|  |_|______|_| `_|
///
/// ===================================================================================================================
///
/// RAMEN (Real-time Actor-based Message Exchange Network) is a very compact single-header C++20+ dependency-free
/// library that implements message-passing computation semantics for hard real-time mission-critical embedded systems.
/// It is designed to be very low-overhead, efficient, and easy to use. Please refer to the README for the docs.
///
/// The library includes the core functionality (the message passing ports) along with a few utilities for
/// practical message-passing programs.
///
/// The notation to define control flow, data flow, and relation between components is described here.
///
/// In diagrams, arrows represent the control flow direction: the pointed-to item is invoked by the pointing item.
/// Actors accept data inputs from the left and output data to the right.
/// A control input (the invoked item) is called a behavior, and a control output (like a callback) is a delegate.
/// Overall this results in four possible combinations of input/output ports,
/// which can be used to arrange two dataflow models -- push (eager) and pull (lazy):
///
///     Port kind       Control     Data    Alias
///     --------------------------------------------
///      in-behavior    in          in      Pushable
///     out-delegate    out         out     Pusher
///     out-behavior    in          out     Pullable
///      in-delegate    out         in      Puller
///
/// Pull-model ports pair naturally with other pull-model ports, and the same for push-model ports.
/// To bridge push and pull models together, we use two basic components:
/// Latch (with behaviors on either side) and Lift (with delegates on either side and an additional trigger behavior);
/// they are defined below.
///
/// The following diagramming notation is adopted:
///
///                                +--------+
///   (input behavior) pushable -->|        |--> pusher (output delegate)
///                                | Actor  |
///   (input delegate) puller   <--|        |<-- pullable (output behavior)
///                                +--------+
///
/// The type of exchanged data can be arbitrary and it is possible to exchange more than one object per port.
/// Behaviors and Delegates can be linked together arbitrarily using operator>>; remember that the arrows point in the
/// direction of the control flow, not data flow. Within an actor, a behavior may trigger some delegates;
/// a delegate of an actor can trigger behaviors in other actors that it is linked to.
/// In some scenarios, the direction of the control flow does not matter on a bigger scale,
/// in which case one can use operator^ to link behaviors and delegates together irrespective of the control direction.
///
/// Delegates can be linked with behaviors and other delegates into topics. Given multiple delegates on a topic,
/// the resulting behavior is that any of the linked delegates will cause all linked behaviors to be triggered in
/// the order of linking. Topics that do not contain behaviors have no effect (delegates do not affect each other).
/// Behaviors cannot be linked directly without delegates.
///
/// Actors are usually implemented as structs with all data fields public. Public data does not hinder
/// encapsulation because actors are unable to affect or even see each other's data directly, as all interation is
/// done through message passing; this is the essence of the actor model. Hence, the encapsulation mechanisms provided
/// by C++ become redundant. This point only holds for pure actors, however; there may exist mixed classes that work
/// both as actors and as regular objects, in which case this recommendation may not apply
/// (whether it's a good practice to define such mixed classes is another matter).
///
/// Remember that recursive dependencies are common in actor networks, especially when they are used to implement
/// control systems. When implementing an actor, keep in mind that triggering any delegate to push or pull data can
/// cause the control flow to loop back to the current actor through a possibly very long chain of interactions.
/// Proper design should prevent the possibility of descending into an infinite recursion and also serving data
/// from an actor whose internal state is inconsistent. To avoid the latter class of errors, state updates should
/// always be performed in a transactional manner: first, all inputs are read, then the state is updated, and only
/// then the outputs are written. This is a general rule of thumb for designing such systems and is not specific
/// to this library.
///
/// Pushable and Pusher accept const references to the passed data. It is not possible to pass non-const
/// references nor rvalue references because the data may be passed through a chain of actors, and it is necessary
/// to guarantee that each will receive the exact same data regardless of the order in which the actors are invoked.
///
/// Puller and Pullable accept mutable references to the data because the data is returned via out-parameters.
/// This is necessary to support the use case when the source of the data does not know the size of
/// the returned data statically and at the same time the use of the dynamic memory is not allowed. One specific case
/// where this situation arises is Eigen::MatrixRef<> with at least one dynamic dimension.
///
/// Defining ports of highly specialized types is possible but rarely useful because specialized types impair
/// composability. For example, suppose there is a specialized configuration struct for some actor. The actor could
/// accept the configuration via an in-port and it would work, but the utility of this choice is limited because
/// in order to make use of this port, the other actor would need to have access to its specific type, at which point
/// the message passing aspect becomes redundant, as it would be easier to just pass/alter the configuration struct
/// directly (e.g., by mutating the state of the first actor). However, the first actor were to accept configuration
/// via more granular in-ports of more generic types, like vectors, matrices, or whatever is common in the application,
/// then the composability of the solution would not be compromised.
///
/// Message passing enables a new approach to policy-based design. It is possible to ship predefined policies with
/// an actor by defining several behaviors, each implementing its own policy while sharing the same type.
/// The client will then choose which particular behavior to link with their own delegates.
///
/// ===================================================================================================================
///
/// Author: Pavel Kirienko <pavel.kirienko@zubax.com>
///
/// MIT License
///
/// Copyright (c) Zubax Robotics
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
/// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
/// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions
/// of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
/// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
/// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
/// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include <array>
#include <tuple>
#include <cassert>
#include <cstddef>
#include <algorithm>
#include <type_traits>

namespace ramen
{
/// An invokable entity with the specified function signature.
template <typename>
class Callable;

/// This is like std::function but without heap, move-only, and non-default-constructible such that it is always valid.
/// The size of the callable target is limited by the footprint; an attempt to exceed it will trigger a compile error.
/// For safety reasons, it is impossible to construct an invalid Function -- there is no default constructor on purpose.
/// The underlying callable entity must be move-constructible. Copy ctor and copy/move assignment are not required.
/// The function is move-constructible and move-assignable.
/// A function can be move-constructed or move-assigned from another function with a different footprint and/or
/// alignment, as long as the new footprint and alignment are not larger than the original ones.
template <typename, std::size_t footprint, std::size_t alignment = alignof(std::max_align_t)>
class Function;

/// This type is used to change the default function footprint when needed.
/// Refer to Pushable/Pullable etc for usage details.
template <std::size_t size>
struct Footprint;

/// The default lambda footprint is enough to capture one pointer (e.g., this). If needed, it can be increased ad-hoc.
constexpr std::size_t default_behavior_footprint = std::max({sizeof(void(std::tuple<>::*)()),  //
                                                             sizeof(void*),
                                                             sizeof(void (*)())});

/// A behavior is a control input that is used either for pulling or pushing data. It contains a user-defined
/// function that can be linked with other behaviors and delegates into a multicast call chain.
/// A group of behaviors and delegates linked together form a topic.
///
/// ATTENTION: If the lambda captures `this`, the entire object should be non-copyable and non-movable;
/// otherwise, the captured this pointer will be dangling after moving, or point to the wrong object after copying.
/// Instances of Behavior are always non-copyable, so this is not a problem. By default, moving is disabled as well for
/// safety reasons; however, if the lambda is safe to move (which means it doesn't capture this;
/// capturing fields by value is fine though), moving can be enabled via the last template parameter.
template <typename, std::size_t footprint, bool movable = false>
struct Behavior;

/// A delegate is a control output that is used either for pulling or pushing data. It is an interface for invoking
/// behaviors that belong to the same topic. By themselves, delegates have no effect when triggered until linked
/// with at least one behavior.
/// An attempt to link a delegate or a behavior to the same topic more than once has no effect.
template <typename>
struct Delegate;

// ====================================================================================================================

template <typename R, typename... A>
class Callable<R(A...)>
{
public:
    template <typename F>
    constexpr static bool is_compatible = std::is_invocable_r_v<R, F, A...>;

    [[nodiscard]] virtual R operator()(A... args) const = 0;

    Callable() noexcept                      = default;
    Callable(const Callable&)                = default;
    Callable(Callable&&) noexcept            = default;
    Callable& operator=(const Callable&)     = default;
    Callable& operator=(Callable&&) noexcept = default;

protected:
    ~Callable() noexcept = default;
};

namespace detail
{
template <typename Signature, std::size_t footprint, std::size_t alignment, typename Target>
struct IsValidTarget : public std::false_type
{
};
// A Function is never a valid target for another Function. We support assignment for that.
template <typename X, typename Y, std::size_t a, std::size_t b, std::size_t c, std::size_t d>
struct IsValidTarget<X, a, b, Function<Y, c, d>> : public std::false_type
{
};
template <typename F, std::size_t fp, std::size_t al, typename T>
requires((Callable<F>::template is_compatible<T>) && (sizeof(T) <= fp) && (alignof(T) <= al))
struct IsValidTarget<F, fp, al, T> : public std::is_move_constructible<T>
{
};
}  // namespace detail

template <typename R, typename... A, std::size_t footprint, std::size_t alignment>
class Function<R(A...), footprint, alignment> final : public Callable<R(A...)>
{
    template <typename, std::size_t, std::size_t>
    friend class Function;

    static_assert((alignment & (alignment - 1)) == 0, "alignment must be a power of 2");

public:
    /// This function can be instantiated with any callable F for which this value is true.
    template <typename F, typename T = std::decay_t<F>>
    constexpr static bool is_valid_target = detail::IsValidTarget<R(A...), footprint, alignment, T>::value;
    /// Accepts a callable object and move-constructs it into the function.
    /// The new callable must meet is_valid_target; otherwise, this ctor does not participate in overload resolution.
    template <typename F>
    requires is_valid_target<F>                 // NOLINTNEXTLINE(*-explicit-*)
    explicit(false) Function(F&& fun) noexcept  // NOSONAR constraint is enforced by the requires clause
    {
        construct(std::forward<F>(fun));
    }
    Function(const Function& that) = delete;
    Function(Function&& that) noexcept : call_(that.call_), dtor_(that.dtor_), move_(that.move_)
    {
        assert(move_ != nullptr);
        move_(fun_.data(), that.fun_.data());
    }
    template <std::size_t foot2, std::size_t align2>
    requires(foot2 <= footprint) && (align2 <= alignment)
    // NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved,*-explicit-*)
    explicit(false) Function(Function<R(A...), foot2, align2>&& that) :
        call_(that.call_),
        dtor_(that.dtor_),
        move_(that.move_)
    {
        assert(move_ != nullptr);
        move_(fun_.data(), that.fun_.data());
    }

    Function& operator=(const Function& that) = delete;
    Function& operator=(Function&& that) noexcept
    {
        if (this != &that)
        {
            assign(std::move(that));
        }
        return *this;
    }
    template <std::size_t foot2, std::size_t align2>
    requires(foot2 <= footprint) && (align2 <= alignment)
    Function& operator=(Function<R(A...), foot2, align2>&& that) noexcept
    {
        assign(std::move(that));
        return *this;
    }
    /// Allows replacing the underlying callable object with a new one. The new callable must meet is_valid_target;
    /// otherwise, this operator does not participate in overload resolution.
    template <typename F>
    requires is_valid_target<F>
    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature,misc-unconventional-assign-operator)
    Function& operator=(F&& fun) noexcept
    {
        destroy();
        construct(std::forward<F>(fun));
        return *this;
    }

    [[nodiscard]] R operator()(A... args) const final { return call_(fun_.data(), args...); }

    ~Function() noexcept { destroy(); }  // This is not virtual because the class is final.

private:
    template <typename F, typename T = std::decay_t<F>>
    requires is_valid_target<T>
    void construct(F&& fun) noexcept
    {
        assert(dtor_ == nullptr);
        // NOSONARBEGIN reinterpret_cast, void*
        // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
        call_ = [](void* const ptr, A... args) -> R { return (*reinterpret_cast<T*>(ptr))(args...); };
        dtor_ = [](void* const ptr) noexcept { reinterpret_cast<T*>(ptr)->~T(); };
        move_ = [](void* const dst, void* const src) noexcept { new (dst) T(std::move(*reinterpret_cast<T*>(src))); };
        // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
        // NOSONAREND
        static_assert((sizeof(T) <= footprint) && (alignof(T) <= alignment));
        (void) new (fun_.data()) T(std::forward<F>(fun));  // NOSONAR placement new does not allocate memory.
    }

    void destroy() noexcept
    {
        assert(dtor_ != nullptr);
        dtor_(fun_.data());
        dtor_ = nullptr;
        call_ = nullptr;
        move_ = nullptr;
    }

    template <std::size_t foot2, std::size_t align2>
    requires(foot2 <= footprint) && (align2 <= alignment)
    // NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved)
    void assign(Function<R(A...), foot2, align2>&& that) noexcept
    {
        destroy();
        assert(dtor_ == nullptr);
        call_ = that.call_;
        dtor_ = that.dtor_;
        move_ = that.move_;
        assert(move_ != nullptr);
        move_(fun_.data(), std::move(that).fun_.data());
    }

    alignas(alignment) mutable std::array<unsigned char, footprint> fun_;
    // NOSONARBEGIN void*
    R (*call_)(void*, A...)              = nullptr;
    void (*dtor_)(void*) noexcept        = nullptr;
    void (*move_)(void*, void*) noexcept = nullptr;
    // NOSONAREND
};

// ====================================================================================================================
namespace detail
{
/// An ordinary double-linked list node.
/// To use it, inherit from it and pass the base class as a template argument.
template <typename T>
class ListNode : public T
{
public:
    template <typename... Args>
    requires((sizeof...(Args) != 1) ||  // Ensure this ctor doesn't match on copy/move.
             (!std::is_same_v<ListNode, std::decay_t<std::tuple_element_t<0, std::tuple<Args...>>>>) )
    explicit ListNode(Args&&... args) : T(std::forward<Args>(args)...)
    {
    }
    template <typename Arg>
    requires(!std::is_same_v<std::decay_t<Arg>, ListNode>)
    explicit ListNode(Arg&& arg) : T(std::forward<Arg>(arg))
    {
    }
    ListNode(const ListNode&) = delete;
    ListNode(ListNode&& that) noexcept { operator=(std::move(that)); }
    ListNode& operator=(const ListNode&) = delete;
    ListNode& operator=(ListNode&& that) noexcept
    {
        if (this != &that)
        {
            remove();
            prev_ = that.prev_;
            next_ = that.next_;
            if (prev_ != nullptr)
            {
                prev_->next_ = this;
            }
            if (next_ != nullptr)
            {
                next_->prev_ = this;
            }
            that.prev_ = nullptr;
            that.next_ = nullptr;
        }
        return *this;
    }

    [[nodiscard]] bool      linked() const noexcept { return (prev_ != nullptr) || (next_ != nullptr); }
    [[nodiscard]] ListNode* next() const noexcept { return next_; }
    [[nodiscard]] ListNode* head() noexcept
    {
        ListNode* p = this;
        while (p->prev_ != nullptr)
        {
            p = p->prev_;
        }
        return p;
    }
    [[nodiscard]] ListNode* tail() noexcept
    {
        ListNode* p = this;
        while (p->next_ != nullptr)
        {
            p = p->next_;
        }
        return p;
    }

    /// Merges two linked lists such that the list that contains "that" is appended to "this".
    /// Does nothing if this and that are already members of the same list (this is done through linear traversal).
    /// The time complexity is linear of the total number of elements in both lists.
    void merge(ListNode* const that) noexcept
    {
        if (that != nullptr)
        {
            ListNode* const that_head = that->head();
            if (this->head() != that_head)  // Also weeds out the case when this==that.
            {
                ListNode* const this_tail = this->tail();
                this_tail->next_          = that_head;
                that_head->prev_          = this_tail;
            }
        }
    }

    /// This is a form of O(n*k) sorting where each Node has a clustering key value and the nodes are arranged
    /// according to the value of their clustering key (n is the Node count, k is the cluster count).
    /// The list is always rewound to the beginning before commending the clusterization.
    /// The order of the nodes with the same key value is preserved.
    /// The clustering key is an std::size_t value that is calculated by the key function.
    /// Behavior undefined if the key function returns a value that is out of the range [0, ClusterCount).
    template <std::size_t N, typename K>
    requires std::is_invocable_r_v<std::size_t, K, const ListNode&>
    void clusterize(const K& key) noexcept
    {
        // Make a single pass over the list, splitting it into several lists by clustering key.
        std::array<std::pair<ListNode*, ListNode*>, N> clusters{};
        ListNode*                                      p = this->head();
        do
        {
            ListNode* const next = p->next_;
            p->remove();
            if (std::pair<ListNode*, ListNode*>& c = clusters.at(key(*p)); c.first == nullptr)
            {
                c.first  = p;
                c.second = p;
            }
            else
            {
                assert((nullptr == c.first->prev_) && (nullptr == c.second->next_) && (nullptr == p->prev_));
                c.second->next_ = p;
                p->prev_        = c.second;
                c.second        = p;
            }
            p = next;
        } while (p != nullptr);
        // Merge the lists back together.
        std::pair<ListNode*, ListNode*> x{};
        for (const auto& c : clusters)
        {
            if (c.first != nullptr)  // Skip empty lists.
            {
                if (x.first != nullptr)
                {
                    x.second->next_ = c.first;
                    c.first->prev_  = x.second;
                    x.second        = c.second;
                }
                x = c;
            }
        }
    }

    void remove() noexcept
    {
        if (prev_ != nullptr)
        {
            prev_->next_ = next_;
        }
        if (next_ != nullptr)
        {
            next_->prev_ = prev_;
        }
        prev_ = nullptr;
        next_ = nullptr;
    }

protected:
    ~ListNode() noexcept { remove(); }

private:
    ListNode* prev_ = nullptr;
    ListNode* next_ = nullptr;
};

template <typename>
class Triggerable;
template <typename R, typename... A>
class Triggerable<R(A...)>
{
public:
    virtual R trigger(A... args) const = 0;

    [[nodiscard]] virtual std::size_t key() const noexcept = 0;

    Triggerable() noexcept                         = default;
    Triggerable(const Triggerable&)                = default;
    Triggerable(Triggerable&&) noexcept            = default;
    Triggerable& operator=(const Triggerable&)     = default;
    Triggerable& operator=(Triggerable&&) noexcept = default;

protected:
    ~Triggerable() noexcept = default;
};

/// It is essential that the inheritance from the ListNode is protected. Public inheritance would spill out the
/// internal implementation details from T. Inheritors have to use public inheritance of this type because their
/// references must be implicitly convertible into this type for topic linking.
template <typename T>
struct Port : protected ListNode<T>
{
    using ListNode<T>::linked;

    /// The operator bool shall be explicit to avoid unintentional conversions when using the linking operators
    /// like >> or ^ with unlinkable entities, as it would make the code syntactically valid but meaningless.
    explicit constexpr           operator bool() const noexcept { return this->linked(); }
    [[nodiscard]] constexpr bool operator!() const noexcept { return !this->linked(); }

    Port()                           = default;
    Port(const Port&)                = delete;
    Port(Port&&) noexcept            = default;
    Port& operator=(const Port&)     = delete;
    Port& operator=(Port&&) noexcept = default;

protected:
    ~Port() noexcept = default;
};

template <bool>
struct EnableCopyMove;
template <>
struct EnableCopyMove<true>
{
    EnableCopyMove() noexcept                            = default;
    EnableCopyMove(EnableCopyMove&&) noexcept            = default;
    EnableCopyMove(const EnableCopyMove&)                = default;
    EnableCopyMove& operator=(EnableCopyMove&&) noexcept = default;
    EnableCopyMove& operator=(const EnableCopyMove&)     = default;
    ~EnableCopyMove() noexcept                           = default;
};
template <>
struct EnableCopyMove<false>
{
    EnableCopyMove() noexcept                            = default;
    EnableCopyMove(EnableCopyMove&&) noexcept            = delete;
    EnableCopyMove(const EnableCopyMove&)                = delete;
    EnableCopyMove& operator=(EnableCopyMove&&) noexcept = delete;
    EnableCopyMove& operator=(const EnableCopyMove&)     = delete;
    ~EnableCopyMove() noexcept                           = default;
};
}  // namespace detail

// ====================================================================================================================

template <typename R, typename... A, std::size_t footprint, bool movable>
struct Behavior<R(A...), footprint, movable> : public detail::Port<detail::Triggerable<R(A...)>>,
                                               private detail::EnableCopyMove<movable>
{
    template <typename F>
    requires((Function<R(A...), footprint>::template is_valid_target<F>) && (!std::is_same_v<F, Behavior>) )
    Behavior(F&& fun) : fun_(std::forward<F>(fun))  // NOLINT(*-explicit-*) NOSONAR does not match on copy/move
    {
    }
    Behavior(const Behavior&)                = delete;
    Behavior(Behavior&&) noexcept            = default;
    Behavior& operator=(const Behavior&)     = delete;
    Behavior& operator=(Behavior&&) noexcept = default;

    virtual ~Behavior() noexcept = default;

    /// Behaviors are NOT meant to be invoked directly except in very special cases.
    /// The normal usage pattern is to link them up with delegates and other behaviors into topics,
    /// and then trigger topics via delegates as needed.
    ///
    /// In certain special cases, however, one may want to invoke a behavior directly; this is how it is done.
    /// The direct invocation is slow (linear complexity of the number of items in the topic).
    /// Normally, one should limit direct invocations only for the initial configuration stage, where the configuration
    /// objects need to be distributed to the actors once.
    ///
    /// Obviously, if the directly invoked behavior is a part of a topic, the entire topic will be triggered as well.
    [[nodiscard]] R operator()(A... args)
    {
        // One might be tempted to simply rewind the list to the beginning and trigger the first item on it,
        // because the list is sorted such that the delegates are in the beginning. This won't work if there are no
        // behaviors on this topic, though. The current implementation is a bit slower but is robust.
        // We could add a special case for unlinked behaviors, though: if (!*this) { return trigger(args...); }
        Delegate<R(A...)> delegate;  // The delegate will be destroyed and unlinked at the end.
        delegate >> *this;
        return delegate(args...);
    }

private:
    R trigger(A... args) const final
    {
        // We know the concrete type of fun and its operator() is final, so there is no virtual call overhead:
        // the call will be inlined. In the end, the entire invocation chain is as follows:
        // the delegate polymorphically calls this->trigger, which inlines the call to fun_(), which then invokes
        // a pointer to the target function; the target function can be inlined into that invoker.
        return fun_(args...);
    }

    [[nodiscard]] std::size_t key() const noexcept final { return 1; }

    Function<R(A...), footprint> fun_;
};

template <typename... A>
struct Delegate<void(A...)> : public detail::Port<detail::Triggerable<void(A...)>>
{
    using Port = detail::Port<detail::Triggerable<void(A...)>>;

    /// A delegate is true if it is linked with anything, false otherwise.
    /// Some actors may choose to skip computations associated with unlinked delegates to reduce the execution time.
    using Port::operator bool;
    using Port::operator!;

    Delegate() noexcept                      = default;
    Delegate(const Delegate&)                = delete;
    Delegate(Delegate&&) noexcept            = default;
    Delegate& operator=(const Delegate&)     = delete;
    Delegate& operator=(Delegate&&) noexcept = default;

    /// Links this delegate with a behavior or another delegate such that when any delegate on the topic is triggered,
    /// all linked behaviors will be triggered, too.
    /// The order of linking is important: the linked behaviors will be triggered in the order of linking.
    /// If the other is already a member of another topic, the topics will be merged.
    /// The time complexity of this operation is linear of the number of elements in the topic afterward;
    /// the design is optimized to move the computational cost from the runtime invocation stage to the linking stage.
    Delegate& operator>>(Port& that)
    {
        // Delegates are inserted in the beginning of the topic, while behaviors go to the end. This is needed to
        // avoid rewinding the list to the beginning when commencing topic execution. Since we can effectively join
        // two topics together at any moment, we have to sort the linked list after every insertion.
        this->merge(&that);
        this->template clusterize<2>([](const detail::Triggerable<void(A...)>& x) { return x.key(); });
        return *this;
    }

    void operator()(A... args) const
    {
        // The list is kept sorted such that the delegates are in the beginning. This allows us to start traversal from
        // the next (!) node without the need to rewind first; at the same time we can skip some of the initial
        // delegates. A more interesting design would keep two independent linked lists: a list of ports and a list of
        // behaviors. This will remove unnecessary calls to the empty trigger() function if there is more than one
        // delegate on the topic.
        detail::ListNode<detail::Triggerable<void(A...)>>* p = this->next();
        while (p != nullptr)
        {
            p->trigger(args...);
            p = p->next();
        }
    }

    /// Removes this delegate from the topic; undoes the effect of operator>>.
    void detach() noexcept { this->remove(); }

    virtual ~Delegate() noexcept = default;

private:
    using Fun = void(A...);

    void trigger(A...) const final
    {
        // This is a no-op because delegates do not have a direct execution behavior. This approach would not work in
        // the case of a non-void return type; there, a slightly more sophisticated handling is needed. Perhaps the
        // delegate does not need to be Triggerable at all, nor exist on the same hierarchy level as behaviors.
        // This can be redesigned in the future without altering the user interface.
    }

    [[nodiscard]] std::size_t key() const noexcept final { return 0; }

    /// This operator is a trivial wrapper over Delegate::operator>> used as a convenience helper when one doesn't
    /// care about the direction of the control flow and just needs to link two or more ports together.
    /// The operator can be chained like in1^in2^out1^out2.
    /// This operator is chosen because it looks like an arrow that points neither left nor right, and also because
    /// it is used very infrequently.
    friend Delegate& operator^(Delegate& le, detail::Port<detail::Triggerable<Fun>>& ri) noexcept { return le >> ri; }
    friend Delegate& operator^(detail::Port<detail::Triggerable<Fun>>& le, Delegate& ri) noexcept { return ri >> le; }
    // This overload is only needed to avoid the ambiguity when both arguments are delegates.
    friend Delegate& operator^(Delegate& le, Delegate& ri) noexcept { return le >> ri; }
};
/// Delegates with non-void return type are not supported.
/// The support can be added without much trouble but right now there doesn't seem to be an apparent use case for that.
template <typename R, typename... A>
struct Delegate<R(A...)>;

// ====================================================================================================================

/// A pushable is an in-behavior, which represents an input port for the push model.
/// It sources the value from one or more of the connected out-delegates.
///
///        +--------+
///     -->| actor  |
///        +--------+
///
/// Usage:
///     Pushable<T> in_abc = [this](const T& value) { ... };
///
/// Multiple-input pushables can be defined:
///     Pushable<A, B, C> in_abc = [this](const A& a, const B& b, const C& c) { ... };
///
/// Valueless pushables can be used to implement side effects (the void specialization is introduced for generality
/// when T is deduced from some function return type):
///     Pushable<>     in_trigger = [this] { ... };
///     Pushable<void> in_trigger = [this] { ... };  // equivalent to the above
///
/// To change the default footprint for the callable, wrap it into Footprint<size> and pass as the first argument:
///     Pushable<Footprint<128>, A, B, C> in_abc = [this, that, other](const A& a, const B& b, const C& c) { ... };
template <typename... T>
struct Pushable final : public Behavior<void(const T&...), default_behavior_footprint>
{
    using Behavior<void(const T&...), default_behavior_footprint>::Behavior;
};
template <typename... T, std::size_t fp>
struct Pushable<Footprint<fp>, T...> final : public Behavior<void(const T&...), fp>
{
    using Behavior<void(const T&...), fp>::Behavior;
};
template <>
struct Pushable<void> final : public Behavior<void(), default_behavior_footprint>
{
    using Behavior<void(), default_behavior_footprint>::Behavior;
};
template <std::size_t fp>
struct Pushable<Footprint<fp>, void> final : public Behavior<void(), fp>
{
    using Behavior<void(), fp>::Behavior;
};

/// A Pusher is an out-delegate, which represents an output port for the push model.
/// It supplies the value to one or more of the connected in-behaviors.
/// Multiple out-delegates can be used on the same topic with one or more in-behaviors without affecting each other.
///
///        +--------+
///        | actor  |-->
///        +--------+
///
/// The void specialization is introduced for generality when T is deduced from some function return type:
/// Pusher<> and Pusher<void> are equivalent.
template <typename... T>
struct Pusher final : public Delegate<void(const T&...)>
{
    using Delegate<void(const T&...)>::operator();
    using Delegate<void(const T&...)>::operator>>;
};
template <>
struct Pusher<void> final : public Delegate<void()>
{
    using Delegate<void()>::operator();
    using Delegate<void()>::operator>>;
};

/// A Pullable is an out-behavior, which represents an output port for the pull model.
/// It supplies the value to one or more of the connected in-delegates.
///
///        +--------+
///        | actor  |<--
///        +--------+
///
/// Usage:
///     Pullable<T> out_abc = [this](T& out_value) { ... };
///
/// Multiple-output pullables can be defined:
///     Pullable<A, B, C> out_abc = [this](A& out_a, B& out_b, C& out_c) { ... };
///
/// To change the default footprint for the callable, wrap it into Footprint<size> and pass as the first argument:
///     Pullable<Footprint<128>, A, B> out_abc = [this](A& out_a, B& out_b) { ... };
template <typename... T>
struct Pullable final : public Behavior<void(T&...), default_behavior_footprint>
{
    using Behavior<void(T&...), default_behavior_footprint>::Behavior;
};
template <typename... T, std::size_t fp>
struct Pullable<Footprint<fp>, T...> final : public Behavior<void(T&...), fp>
{
    using Behavior<void(T&...), fp>::Behavior;
};

/// A Puller is an in-delegate, which represents an input port for the pull model.
/// It sources the value from one or more of the connected out-behaviors.
/// Multiple in-delegates can be used on the same topic with one or more out-behaviors without affecting each other.
///
///        +--------+
///     <--| actor  |
///        +--------+
///
/// The void specialization is introduced for generality when T is deduced from some function return type:
/// Pusher<> and Pusher<void> are equivalent.
template <typename... T>
struct Puller final : public Delegate<void(T&...)>
{
    using Delegate<void(T&...)>::operator();
    using Delegate<void(T&...)>::operator>>;
};
/// The specialization for a single default-constructible T is equipped with fancy helpers.
template <typename T>
requires std::is_default_constructible_v<T>
struct Puller<T> final : public Delegate<void(T&)>
{
    using Delegate<void(T&)>::operator();
    using Delegate<void(T&)>::operator>>;
    /// A fancy helper for sourcing the value.
    T operator*() const
    {
        T out{};
        operator()(out);
        return out;
    }
    /// A fancy helper for sourcing the value.
    auto operator->() const
    {
        struct
        {
            T  value{};
            T* operator->() noexcept { return &value; }
        } out;
        operator()(out.value);
        return out;
    }
};
template <>
struct Puller<void> final : public Delegate<void()>
{
    using Delegate<void()>::operator();
    using Delegate<void()>::operator>>;
};

// ====================================================================================================================

/// A Latch<T> bridges a push-model output with a pull-model input, acting as a one-element-deep queue.
/// By default, the input and output types are the same as T, but this can be overridden, in which case the
/// latch will perform a static_cast from In to T upon input and from T to Out upon output.
///
///                               +-------+
///                         (In)  |       |  (Out)
///  (input behavior) pushable -->| Latch |<-- pullable (output behavior)
///                               |       |
///                               +-------+
///
/// As it has to expose a push-input and a pull-output, it has behaviors on both sides and no delegates.
/// The Latch has a default value that is written on push and read on pop; it can be changed by the user as well.
///
/// Do not use Latch to define actor ports; actors usually should only provide {Push,Pull}{able,er} ports.
/// Latches are normally needed only when linking actors together at the place of use.
template <typename T, typename In = T, typename Out = T>
struct Latch
{
    /// The value should be initialized once and left intact. It will updated every time the Latch is pushed.
    T value{};

    Pushable<In>  in  = [this](const In& val) { value = static_cast<T>(val); };
    Pullable<Out> out = [this](Out& val) { val = static_cast<Out>(value); };
};

/// A Lift<T> bridges a pull-model output with a push-model input.
/// By default, the output type is the same as T, but this can be overridden, in which case the Lift will
/// perform a static_cast from T to Out upon output.
///
///                                +-------+
///                           (T)  |       |  (Out)
///     (input delegate) puller <--| Lift  |--> pusher (output delegate)
///                            ()  |       |
///   (input behavior) pushable -->|       |
///                                |       |
///                                +-------+
///
/// As it has to expose a pull-input and a push-output, it has delegates on both sides.
/// Since the pull-input is lazy, it has to receive an external trigger to pull the input and then push it out;
/// this is done via an empty input behavior.
template <typename T, typename Out = T>
struct Lift
{
    /// The value should be initialized once and left intact. It will updated every time the Lift is triggered.
    T value{};

    Puller<T>   in{};
    Pusher<Out> out{};

    Pushable<> trigger = [this]
    {
        in(value);
        out(static_cast<Out>(value));
    };
};

// ====================================================================================================================

/// Invokes the user-provided function Out(const In&...) on every pushed input In... and pushes the result Out to the
/// output. It is called unary because it accepts only one input; the input itself, however, can have an arbitrary
/// number of elements.
///
///                                +-----------+
///                        (In...) |           |  (Out)
///   (input behavior) pushable -->| PushUnary |--> pusher (output delegate)
///                                |           |
///                                +-----------+
///
/// The function is wrapped into the in-behavior handler to avoid an extra call indirection compared to the case
/// when the function is stored in a separate Function<> field.
///
/// Usage:
///     PushUnary<double, int> fun = [](int val) { return static_cast<double>(val) + 123; };
///
/// To change the default footprint for the callable, wrap it into Footprint<size> and pass as the first argument:
///     PushUnary<Footprint<128>, double, int> fun = [](int val) { return static_cast<double>(val) + 123; };
template <typename Out, typename... In>
struct PushUnary;
template <std::size_t fp, typename Out, typename... In>
struct PushUnary<Footprint<fp>, Out, In...>
{
    template <typename F>
    requires std::is_invocable_r_v<Out, F, In...>  // NOLINTNEXTLINE(*-explicit-*)
    PushUnary(F fun) :
        in(
            [this, fun_ = std::move(fun)](const In&... val)
            {
                if constexpr (std::is_same_v<Out, void>)
                {
                    fun_(val...);
                    out();
                }
                else
                {
                    out(fun_(val...));
                }
            })
    {
    }
    Pushable<Footprint<sizeof(std::tuple<std::array<std::byte, fp>, void*>)>, In...> in;
    Pusher<Out>                                                                      out;
};
template <typename Out, typename... In>
struct PushUnary : public PushUnary<Footprint<default_behavior_footprint>, Out, In...>
{
    using PushUnary<Footprint<default_behavior_footprint>, Out, In...>::PushUnary;
};

// TODO: add PushNary --- waits for all push inputs to be updated and then triggers the push output once.

/// Pulls the input In..., invokes the user-provided function Out(In...), and returns output Out on every output pull.
/// It is called unary because it accepts only one input; the input itself, however, can have an arbitrary number
/// of elements.
///
///                                 +-----------+
///                         (In...) |           |  (Out)
///      (input delegate) puller <--| PullUnary |<-- pullable (output behavior)
///                                 |           |
///                                 +-----------+
///
/// The 'value' field is a temporary that is used to store the input values before they are passed to the function.
/// It is exposed to support the case when at least one member of In... is not default-constructible.
/// The 'value' is a tuple of In... unless In is a single type, in which case the type of 'value' is just In.
///
/// The function is wrapped into the in-behavior handler to avoid an extra call indirection compared to the case
/// when the function is stored in a separate Function<> field.
///
/// Usage:
///     PullUnary<double, int> fun = [](int val) { return static_cast<double>(val) + 123; };
///
/// If at least one In... is not default-constructible:
///     PullUnary<double, A, B> fun{
///         [](...) { ... },
///         initial_a,
///         initial_b,
///     };
///
/// Zero-argument functions are also supported for generality; they behave like Pullable<Out>:
///     PullUnary<double> fun = [] { return 123.456; };
///
/// To change the default footprint for the callable, wrap it into Footprint<size> and pass as the first argument:
///     PullUnary<Footprint<128>, double, int> fun = [](int val) { return static_cast<double>(val) + 123; };
template <typename Out, typename... In>
struct PullUnary;
template <std::size_t fp, typename Out, typename... In>
struct PullUnary<Footprint<fp>, Out, In...>
{
    template <typename F>
    requires(std::is_invocable_r_v<Out, F, In...> && (sizeof...(In) > 0))
    PullUnary(F&& fun) : PullUnary(std::forward<F>(fun), In{}...)  // NOLINT(*-explicit-*)
    {
    }
    template <typename F>
    requires std::is_invocable_r_v<Out, F, In...>
    PullUnary(F fun, In... initial_values) :  // NOLINT(*-explicit-*)
        value(std::move(initial_values)...),
        out(
            [this, fun_ = std::move(fun)](Out& val)
            {
                std::apply(in, value);
                val = std::apply(fun_, value);
            })
    {
    }
    [[no_unique_address]] std::tuple<In...>                                        value{};
    Puller<In...>                                                                  in{};
    Pullable<Footprint<sizeof(std::tuple<std::array<std::byte, fp>, void*>)>, Out> out;
};
template <std::size_t fp, typename Out, typename In>
struct PullUnary<Footprint<fp>, Out, In>
{
    template <typename F>
    requires std::is_invocable_r_v<Out, F, In>
    PullUnary(F fun, In initial_value = {}) :  // NOLINT(*-explicit-*)
        value(std::move(initial_value)),
        out(
            [this, fun_ = std::move(fun)](Out& val)
            {
                in(value);
                val = fun_(value);
            })
    {
    }
    In                                                                             value;
    Puller<In>                                                                     in{};
    Pullable<Footprint<sizeof(std::tuple<std::array<std::byte, fp>, void*>)>, Out> out;
};
template <typename Out, typename... In>
struct PullUnary : public PullUnary<Footprint<default_behavior_footprint>, Out, In...>
{
    using PullUnary<Footprint<default_behavior_footprint>, Out, In...>::PullUnary;
};

/// Pulls every input In..., invokes the user-provided function Out(In...), and returns output Out on every output pull.
/// This is similar to PullUnary except that for each input argument there is a separate Puller.
/// If there is only one In, the PullNary is equivalent to PullUnary except that the value is always a tuple
/// (if you don't want the tuple, use PullUnary instead).
///
///                                 +-----------+
///                        (In_0)   |           |
///    (input delegate) puller_0 <--|           |
///                                 |           |
///                        (In_1)   |           |  (Out)
///    (input delegate) puller_1 <--| PullNary  |<-- pullable (output behavior)
///                                 |           |
///                        (In_n)   |           |
///    (input delegate) puller_n <--|           |
///                                 |           |
///                                 +-----------+
///
/// The 'value' field is a temporary that is used to store the input values before they are passed to the function.
/// It is exposed to support the case when at least one member of In... is not default-constructible.
///
/// Usage:
///     PullNary<double, int, float> fun = [](int a, float b) { return a + b; };
///
/// If at least one In... is not default-constructible:
///     PullNary<double, A, B> fun{
///         [](...) { ... },
///         initial_a,
///         initial_b,
///     };
///
/// To change the default footprint for the callable, wrap it into Footprint<size> and pass as the first argument.
template <typename Out, typename... In>
struct PullNary;
template <std::size_t fp, typename Out, typename... In>
struct PullNary<Footprint<fp>, Out, In...>
{
    template <typename F>
    requires(std::is_invocable_r_v<Out, F, In...> && (sizeof...(In) > 0))
    PullNary(F&& fun) : PullNary(std::forward<F>(fun), In{}...)  // NOLINT(*-explicit-*)
    {
    }
    template <typename F>
    requires std::is_invocable_r_v<Out, F, In...>
    PullNary(F fun, In... initial_values) :  // NOLINT(*-explicit-*)
        value(std::move(initial_values)...),
        out(
            [this, fun_ = std::move(fun)](Out& val)
            {
                ([this]<std::size_t... Is>(const std::index_sequence<Is...>)
                 { (std::get<Is>(in)(std::get<Is>(value)), ...); })(std::make_index_sequence<sizeof...(In)>{});
                val = std::apply(fun_, value);
            })
    {
    }
    [[no_unique_address]] std::tuple<In...>                                        value;
    [[no_unique_address]] std::tuple<Puller<In>...>                                in{};
    Pullable<Footprint<sizeof(std::tuple<std::array<std::byte, fp>, void*>)>, Out> out;
};
template <typename Out, typename... In>
struct PullNary : public PullNary<Footprint<default_behavior_footprint>, Out, In...>
{
    using PullNary<Footprint<default_behavior_footprint>, Out, In...>::PullNary;
};

// ====================================================================================================================

/// Casts the input from From to To on every push using static_cast. This is a special case of PushUnary.
/// 'To' can be void, which means that the out-delegate will be called without arguments.
template <typename To, typename From>
struct PushCast final : public PushUnary<Footprint<sizeof([] {})>, To, From>
{
    PushCast() : PushUnary<Footprint<sizeof([] {})>, To, From>([](const From& val) { return static_cast<To>(val); }) {}
};
/// Casts the input from From to To on every pull using static_cast. This is a special case of PullUnary.
/// The 'value' field is a temporary that is used to store the input value before it is cast;
/// it is exposed to support the case when From is not default-constructible.
template <typename To, typename From>
struct PullCast final : public PullUnary<Footprint<sizeof([] {})>, To, From>
{
    PullCast() : PullUnary<Footprint<sizeof([] {})>, To, From>([](const From& val) { return static_cast<To>(val); }) {}
};

// ====================================================================================================================

/// A helper for Scala-style constructors that invokes the passed fun immediately and does nothing else.
/// The idea is to enable custom construction-time behaviors while still allowing aggregate initialization syntax.
/// The passed lambda is not stored anywhere, so it has an empty footprint (esp. if [[no_unique_address]] is used).
/// Usage:
///
///     struct my_actor
///     {
///         // ...fields...
///         Ctor _ = [this]{ ...initialization code... };
///     };
struct Ctor final
{
    template <typename F>
    requires std::is_invocable_r_v<void, F>
    Ctor(F&& fun)  // NOLINT(*-explicit-*)  NOSONAR does not match on copy/move due to the concept requirement.
    {
        std::forward<F>(fun)();
    }
};

/// A general-purpose utility: action that is invoked when the object is destroyed. Finalizers are not copyable,
/// but movable. Move assignment disarms the origin and triggers the action in the move target (unless disarmed).
/// The default ctor creates a disarmed finalizer that won't do anything when destroyed, but it can be moved into.
/// Unlike an ordinary dtor, this utility will automatically make the owning object non-copyable, with the correct
/// move-only behavior handling.
///
/// DANGER: Do not attempt to capture this. Capture specific fields strictly by value instead; e.g.:
///     Finalizer<sizeof(void*)> fin_ = [field = this->field]{ ... };
/// While it is valid to capture this, it breaks if the object is moved, because the lambda will still capture the
/// original object, which may no longer exist at the old address. See https://stackoverflow.com/q/78937332/1007777
template <std::size_t footprint = sizeof(void*) * 8>
class Finalizer final
{
public:
    using Fun = Function<void(), footprint>;

    Finalizer() noexcept : Finalizer([] {}) {}

    template <typename F>
    requires(Fun::template is_valid_target<F>)
    Finalizer(F&& action) noexcept :  // NOLINT(*-explicit-*) NOSONAR does not match on copy/move
        act_(std::forward<F>(action))
    {
    }

    Finalizer(const Finalizer&) = delete;
    Finalizer(Finalizer&& that) noexcept : act_(std::move(that.act_))
    {
        that.act_ = [] {};
    }

    Finalizer& operator=(const Finalizer&) = delete;
    Finalizer& operator=(Finalizer&& that) noexcept
    {
        if (this != &that)
        {
            act_();
            act_      = std::move(that.act_);
            that.act_ = [] {};
        }
        return *this;
    }

    ~Finalizer() noexcept { act_(); }

    /// Disengages the finalizer without triggering the action.
    void disarm() noexcept
    {
        act_ = [] {};
    }

private:
    Fun act_;
};

}  // namespace ramen
