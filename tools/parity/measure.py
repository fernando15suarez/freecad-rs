# Print FreeCAD's measurements (fcparity.measure) of $PARITY_FILE as one JSON line.
import json
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.environ.get("PARITY_DIR") or os.getcwd())
import FreeCAD as App  # noqa: E402

from fcparity import measure  # noqa: E402

doc = App.openDocument(os.environ["PARITY_FILE"])
body = next(o for o in doc.Objects if o.TypeId == "PartDesign::Body")
doc.recompute()
print("MEASURE " + json.dumps(measure(doc, body), sort_keys=True))
