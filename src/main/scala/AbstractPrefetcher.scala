package barf

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.subsystem.{CacheBlockBytes}
import firrtl.passes.memlib.Config


sealed trait TLMonitorChannel
object TLMonitorChannel {
  case object A extends TLMonitorChannel
  case object B extends TLMonitorChannel
  case object C extends TLMonitorChannel
  case object D extends TLMonitorChannel
  case object E extends TLMonitorChannel
}



trait CanInstantiatePrefetcher {
  def desc: String
  def instantiate()(implicit p: Parameters): AbstractPrefetcher
}

class SnoopD()(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)
  val data =  UInt(64.W) // bus bytes
  val source = UInt(10.W)
  val done = Bool()
}

class Snoop(snoopD: Boolean = false)(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)
  val snoopDChan = if (snoopD) Some(Valid(new SnoopD())) else None
  val source = if (snoopD) Some(UInt(10.W)) else None
  val write = Bool()
  val address = UInt()
  def block = address >> log2Up(blockBytes)
  def block_address = block << log2Up(blockBytes)
}


class Prefetch(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)

  val write = Bool()
  val address = UInt()
  def block = address >> log2Up(blockBytes)
  def block_address = block << log2Up(blockBytes)
}


case class ConfigSelProjPrefetcher()(implicit val p: Parameters) extends Bundle 
{
  val conf_SelRegion =  UInt(47.W)
  val conf_ProjRegion = UInt(47.W)
  val conf_Routing =    Vec(32, UInt(log2Ceil(16 + 1).W))
  val conf_Operations = Vec(16, Ops())
  val conf_ConstVal =   Vec(16, UInt(32.W))
  val conf_addrRatioNumerator = UInt(4.W)
  val conf_addrRatioDenominator = UInt(4.W)
  val conf_RowBytes = UInt(6.W)
  val conf_selColCount = UInt(4.W)
}

class PrefetcherIO(snoopD: Boolean = false)(implicit p: Parameters) extends Bundle {
  val snoop = Input(Valid(new Snoop(snoopD)))
  val config = if (snoopD) Some(Input(new ConfigSelProjPrefetcher)) else None
  val request = Decoupled(new Prefetch)
  val hit = Output(Bool())
}

abstract class AbstractPrefetcher(val channel: TLMonitorChannel = TLMonitorChannel.A)(implicit p: Parameters) extends Module {
  val io = if (channel == TLMonitorChannel.D) IO(new PrefetcherIO(true)) else IO(new PrefetcherIO)

  io.request.valid := false.B
  io.request.bits := DontCare
  io.request.bits.address := 0.U(1.W)
  io.hit := false.B
}

case class NullPrefetcherParams() extends CanInstantiatePrefetcher {
  def desc() = "Null Prefetcher"
  def instantiate()(implicit p: Parameters) = Module(new NullPrefetcher()(p))
}

class NullPrefetcher(implicit p: Parameters) extends AbstractPrefetcher()(p)

