//go:build !windows
// +build !windows

package cp2112

func checkWriteLength(n int, buf []byte) error {
	if n != len(buf) {
		return ErrSentUnexpectedBytes(n)
	}
	return nil
}
