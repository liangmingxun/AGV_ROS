"""Pure receipt-age and publisher-authority monitor core."""

import math


class SoftwareMonitor:
    def __init__(self, rules, startup_grace):
        if startup_grace < 0.0:
            raise ValueError("startup_grace must be nonnegative")
        self.rules = {}
        for rule in rules:
            topic = str(rule.get("topic", ""))
            maximum_age = float(rule.get("maximum_age", 0.0))
            minimum_publishers = int(rule.get("minimum_publishers", 1))
            maximum_publishers = int(rule.get("maximum_publishers", 1))
            if (not topic.startswith("/") or maximum_age <= 0.0 or
                    minimum_publishers < 0 or
                    maximum_publishers < minimum_publishers):
                raise ValueError("invalid monitor rule: {}".format(rule))
            self.rules[topic] = {
                "maximum_age": maximum_age,
                "minimum_publishers": minimum_publishers,
                "maximum_publishers": maximum_publishers,
                "critical": bool(rule.get("critical", True)),
            }
        if not self.rules:
            raise ValueError("monitor requires at least one rule")
        self.startup_grace = float(startup_grace)
        self.started_at = None
        self.received = {}

    def receive(self, topic, now):
        if topic not in self.rules or not math.isfinite(now):
            return False
        self.received[topic] = now
        return True

    def evaluate(self, now, publisher_map):
        if not math.isfinite(now):
            raise ValueError("monitor time must be finite")
        if self.started_at is None:
            self.started_at = now
        grace = now - self.started_at < self.startup_grace
        entries = {}
        healthy = True
        for topic, rule in self.rules.items():
            stamp = self.received.get(topic)
            age = now - stamp if stamp is not None else math.inf
            count = len(publisher_map.get(topic, []))
            age_ok = grace or age <= rule["maximum_age"]
            authority_ok = (
                rule["minimum_publishers"] <= count <=
                rule["maximum_publishers"])
            ok = age_ok and authority_ok
            if rule["critical"]:
                healthy = healthy and ok
            entries[topic] = {
                "ok": ok,
                "critical": rule["critical"],
                "age": age if math.isfinite(age) else None,
                "maximum_age": rule["maximum_age"],
                "publisher_count": count,
                "publisher_range": [
                    rule["minimum_publishers"],
                    rule["maximum_publishers"],
                ],
            }
        return {"healthy": healthy, "startup_grace": grace, "topics": entries}
