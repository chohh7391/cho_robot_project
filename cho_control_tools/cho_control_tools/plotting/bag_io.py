# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""One topic out of a rosbag2 sqlite3 (.db3) file, for the plot tools."""

from contextlib import closing
import os
import sqlite3


class BagError(Exception):
    """The bag cannot give the plot tool what it reads; the message says why."""


def read_topic(db_path, topic, expected_type, start_t=None, end_t=None):
    """The serialized messages of *topic* as [(seconds since its first message, bytes)].

    Raises BagError when the file is not a rosbag2 sqlite3 bag, the topic is not in
    it, the topic carries a type other than *expected_type*, or nothing is left in
    [start_t, end_t]. Seconds count from the topic's first message, before the
    window is applied.
    """
    if not os.path.isfile(db_path):
        raise BagError(f"'{db_path}' is not a file")
    try:
        with closing(sqlite3.connect(db_path)) as conn:
            topics = dict(conn.execute("SELECT name, type FROM topics").fetchall())
            if topic not in topics:
                same_type = sorted(name for name, type_name in topics.items() if type_name == expected_type)
                raise BagError(
                    f"topic '{topic}' is not in the bag; the bag's {expected_type} topics: "
                    f"{', '.join(same_type) if same_type else 'none'} (ros2 bag info lists them all)")
            if topics[topic] != expected_type:
                raise BagError(f"topic '{topic}' carries {topics[topic]}; this tool reads {expected_type}")
            rows = conn.execute(
                "SELECT timestamp, data FROM messages JOIN topics ON messages.topic_id = topics.id "
                "WHERE topics.name = ? ORDER BY timestamp ASC", (topic,)).fetchall()
    except sqlite3.DatabaseError as error:
        raise BagError(f"'{db_path}' is not a rosbag2 sqlite3 (.db3) bag: {error}") from error

    if not rows:
        raise BagError(f"topic '{topic}' has no messages in the bag")
    first_sec = rows[0][0] / 1e9
    selected = []
    for stamp_ns, data in rows:
        seconds = stamp_ns / 1e9 - first_sec
        if start_t is not None and seconds < start_t:
            continue
        if end_t is not None and seconds > end_t:
            continue
        selected.append((seconds, data))
    if not selected:
        raise BagError(f"topic '{topic}' has no messages between {start_t} s and {end_t} s "
                       f"(it spans 0 s to {rows[-1][0] / 1e9 - first_sec:.2f} s)")
    return selected
