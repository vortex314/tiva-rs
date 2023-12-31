
struct Frame<'a> {
    buffer: &'a mut [u8],
    index: usize,
    max: usize,
    error: Option<Error>,
}

impl Frame {
    fn new(buffer: &mut [u8]) -> Frame {
        Frame { buffer, max:buffer.capacity(),index: 0,None }
    }
    fn sequence_start(&mut self) -> &mut Self {
        self.write(0x7e)
        self
    }
    fn sequence_end(&mut self) -> &mut Self {
        self.buffer[self.index] = 0x7e;
        self.index += 1;
        self
    }

    fn write(&mut self,b:u8) -> &mut Self {
        if self.error.is_some() {
            return self;
        }
        self.index += 1;
        if self.index >= self.max {
            self.error = Some(Error::BufferOverflow);
            return self;
        }
        self.buffer[self.index] = b;
        self
    }

    fn frame_end(&mut self) -> &mut Self {
        self.buffer[self.index] = 0x7e;
        let crc = crc16::State::<crc16::XMODEM>::calculate(&self.buffer[1..self.index]);
        self.write_u16(crc);
        self
    }
    fn frame_begin(&mut self) -> Frame {
        self.write(0x7e)
    }



}

Frame::frame_begin(&buffer).sequence_start().write_u8(1).write_u8(2).write_u8(3).write_u8(4).sequence_end().frame_end();