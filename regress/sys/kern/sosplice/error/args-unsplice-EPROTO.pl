# test EPROTO for unsplicing non-spliced sockets

use strict;
use warnings;
use IO::Socket::IP;
use BSD::Socket::Splice "SO_SPLICE";

our %args = (
    errno => 'EPROTO',
    func => sub {
	my $sl = IO::Socket::IP->new(
	    Proto => "tcp",
	    Listen => 5,
	    LocalAddr => "127.0.0.1",
	) or die "socket listen failed: $!";

	my $s = IO::Socket::IP->new(
	    Proto => "tcp",
	    PeerAddr => $sl->sockhost(),
	    PeerPort => $sl->sockport(),
	) or die "socket failed: $!";

	$s->setsockopt(SOL_SOCKET, SO_SPLICE, pack('i', -1))
	    and die "unsplice non spliced socket succeeded";
    },
);
