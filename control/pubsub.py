import asyncio
import inspect

class Topic:

    def __init__(self, name: str, validator = None):
        self._name = name
        self._validator = validator
        self._subs_async = []
        self._subs_sync = []

    async def publish(self, o):
        if self._validator:
            assert self._validator(o)
        for s in self._subs_sync:
            s(o)
        if self._subs_async:
            await asyncio.wait([s(o) for s in self._subs_async])

    def subscribe(self, s):
        if inspect.iscoroutinefunction(s):
            self._subs_async.append(s)
        else:
            self._subs_sync.append(s)

    def subscribe_async(self, s):
        # not really required
        self._subs_async.append(s)

def s1(o):
    print("s1", o)

def s2(o):
    print("s2", o)

class Foo:
    async def doit(self, o):
        print("foo", o)


async def main():
    def validator(x):
        if type(x) == str:
            return True
        raise ValueError(x)

    t = Topic("topic", validator)
    foo = Foo()
    t.subscribe(s1)
    t.subscribe(s2)
    t.subscribe(foo.doit)
    await t.publish("hello world")
    #await t.publish(1)

if __name__ == "__main__":
    asyncio.run(main())
