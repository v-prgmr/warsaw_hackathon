// Lens Studio JavaScript smoke test. Attach this script to any scene object.
// Assign a Text3D component to the `label` input in the Inspector.
// @input Component.Text3D label

script.createEvent("OnStartEvent").bind(function () {
    if (!script.label) {
        print("[G1 AR test] Assign a Text3D component to the label input.");
        return;
    }

    // Change this string and send the Lens again to test a code update.
    script.label.text = "HELLO G1";
    print("[G1 AR test] HELLO G1 label is ready.");
});
