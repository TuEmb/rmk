use rmk::action::KeyAction;
use rmk::{a, k, layer, mo};
pub(crate) const COL: usize = 4;
pub(crate) const ROW: usize = 4;
pub(crate) const SIZE: usize = 5;
pub(crate) const NUM_LAYER: usize = 2;

#[rustfmt::skip]
pub const fn get_default_keymap() -> [[[KeyAction; COL]; ROW]; NUM_LAYER] {
    [
        layer!([
            [k!(NumLock), k!(Slash), k!(KpAsterisk), k!(Slash)],
            [k!(Kc7), k!(Kc8), k!(Kc9), k!(KpPlus)],
            [k!(Kc4), k!(Kc5), k!(Kc6), k!(KpPlus)],
            [k!(Kc1), k!(Kc2), k!(Kc3), k!(KpEnter)]
            // [k!(Kc0), k!(Kc0), k!(Delete), k!(KpEnter)]
        ]),
        layer!([
            [k!(Kp7), k!(Kp8), k!(Kp9), k!(KpAsterisk)],
            [k!(Kp4), k!(LCtrl), k!(Kp6), k!(KpAsterisk)],
            [mo!(1), k!(Kp2), k!(Kp3), k!(KpAsterisk)],
            [mo!(1), a!(No), k!(Kp0), k!(KpAsterisk)]
            // [mo!(1), a!(No), k!(Kp0), k!(KpAsterisk)]
        ]),
    ]
}
